import numpy as np
from multiprocessing import shared_memory


class IntegrateSystem:

    def __init__(self):
        self.rreach = 0.85  # conserve a bit
        self.rworkspace = 1.0  # operation boundary for the object

        self.qhome = [3.14, -2.1, 1.61, -1.558, -1.562, 0.0]
        self.qhome2 = [1.57, -2.1, 1.61, -1.558, -1.562, 0.0]
        self.qhomeflip = [1.57, -2.3, 2.0, 0.3278, 1.59, np.pi]
        self.gripper_step = 25
        self.gripper_open = 0
        self.gripper_close = 125

        self.tip_to_ee_zoffset = 0.156  # in m dist from robtiq tip to tool0

    def project_point_on_rotated_ellipse3d(
        self,
        center,
        width,
        height,
        depth,
        angle_xy_deg,
        angle_xz_deg,
        target_point,
    ):
        """Project from ellipse center toward target_point onto rotated 3D ellipse boundary."""

        cx, cy, cz = center
        tx, ty, tz = target_point
        a = max(width * 0.5, 1e-9)
        b = max(height * 0.5, 1e-9)
        c = max(depth * 0.5, 1e-9)

        v_world = np.array([tx - cx, ty - cy, tz - cz], dtype=float)
        if np.linalg.norm(v_world) < 1e-12:
            return cx, cy, cz
        theta_xy = np.deg2rad(angle_xy_deg)
        theta_xz = np.deg2rad(angle_xz_deg)
        cth_xy = np.cos(theta_xy)
        sth_xy = np.sin(theta_xy)
        cth_xz = np.cos(theta_xz)
        sth_xz = np.sin(theta_xz)
        # Rotate world vector into ellipse local frame.
        v_local_x = cth_xy * cth_xz * v_world[0] + (
            sth_xy * cth_xz * v_world[1] - sth_xz * v_world[2]
        )
        v_local_y = -sth_xy * v_world[0] + cth_xy * v_world[1]
        v_local_z = cth_xy * sth_xz * v_world[0] + (
            sth_xy * sth_xz * v_world[1] + cth_xz * v_world[2]
        )

        denom = np.sqrt(
            (v_local_x / a) ** 2 + (v_local_y / b) ** 2 + (v_local_z / c) ** 2
        )
        if denom < 1e-12:
            return cx, cy, cz
        p_local = np.array(
            [v_local_x / denom, v_local_y / denom, v_local_z / denom]
        )
        # Rotate boundary point back to world frame.
        px = (
            cth_xy * cth_xz * p_local[0]
            - sth_xy * p_local[1]
            + cth_xy * sth_xz * p_local[2]
            + cx
        )
        py = (
            sth_xy * cth_xz * p_local[0]
            + cth_xy * p_local[1]
            + sth_xy * sth_xz * p_local[2]
            + cy
        )
        pz = -sth_xz * p_local[0] + cth_xz * p_local[2] + cz
        return px, py, pz

    def project_line_to_reach_sphere(self, ee_point, toward_point, radius):
        """Intersect ray from ee_point toward toward_point with sphere centered at origin."""
        ex, ey, ez = ee_point
        tx, ty, tz = toward_point
        d = np.array([tx - ex, ty - ey, tz - ez], dtype=float)
        if np.linalg.norm(d) < 1e-12:
            return ex, ey, ez

        p = np.array([ex, ey, ez], dtype=float)
        a = np.dot(d, d)
        b = 2.0 * np.dot(p, d)
        c = np.dot(p, p) - radius**2
        disc = b * b - 4.0 * a * c

        if disc < 0.0:
            return ex, ey, ez

        sqrt_disc = np.sqrt(disc)
        t1 = (-b - sqrt_disc) / (2.0 * a)
        t2 = (-b + sqrt_disc) / (2.0 * a)
        t_candidates = [t for t in (t1, t2) if t >= 0.0]
        if not t_candidates:
            return ex, ey, ez

        t = max(t_candidates)
        p_hit = p + t * d
        return p_hit[0], p_hit[1], p_hit[2]

    def select_reach_target_model_3d(
        self,
        ee_point,
        ellipse_proj_point,
        radius,
        follow_ellipse,
        hysteresis=0.02,
    ):
        """Select target on reach sphere or ellipse projection using a mode switch.

        Rule:
        - Stay on reach sphere by default.
        - Switch to ellipse projection when ellipse projection enters reach radius.
        - Use small hysteresis to avoid mode chattering near boundary.
        """
        proj_r = np.linalg.norm(ellipse_proj_point)

        enter = proj_r <= radius
        leave = proj_r > radius + hysteresis

        if follow_ellipse:
            if leave:
                follow_ellipse = False
        else:
            if enter:
                follow_ellipse = True

        if follow_ellipse:
            return (
                ellipse_proj_point[0],
                ellipse_proj_point[1],
                ellipse_proj_point[2],
                follow_ellipse,
            )

        xreach, yreach, zreach = self.project_line_to_reach_sphere(
            ee_point=ee_point,
            toward_point=ellipse_proj_point,
            radius=radius,
        )
        return xreach, yreach, zreach, follow_ellipse

    def is_obj_in_boundary(self, pos, boundary_radius):
        """Check if the object is within a certain radius from the origin."""
        distance = np.linalg.norm(pos)
        return distance <= boundary_radius

    def is_obj_ready_to_grasp(self, speed, speed_threshold):
        """Check if the object's speed is below a certain threshold for grasping."""
        return speed <= speed_threshold

    def is_ee_near_obj_for_grasp(self, ee_pos, obj_pos, distance_threshold):
        """Check if the end-effector is within a certain distance from the object for grasping."""
        distance = np.linalg.norm(np.array(ee_pos) - np.array(obj_pos))
        return distance <= distance_threshold

    def is_obj_grasped(self, ftdata):
        """Check if the object is grasped based on force-torque sensor data."""
        pass

    def disingage_after_place(self):
        """Disengage grasp after placing the object."""
        pass

    def keep_grasp_orientation(self, obj_P, reach_P, ee_pose):
        """
        Detect object front/back relative to current EE pose in 3D.

        Generalization from 2D quadrants:
        - 2D front: +x+y, +x-y  -> local x > 0
        - 2D back : -x+y, -x-y  -> local x < 0

        In 3D, use the EE local frame and the sign of local-x of (obj - ee).
        Returns True if object is in front hemisphere, else False.
        """
        ee_P = ee_pose[0:3, 3]
        diff = obj_P - ee_P

        dist = np.linalg.norm(diff)
        if dist < 1e-12:
            return True

        ee_R = ee_pose[0:3, 0:3]
        ee_x = ee_R[:, 0]
        obj_dir = diff / dist

        # Cosine of angle between EE +X and object direction.
        # Use deadband to reduce mode toggling near side boundary.
        front_score = np.dot(ee_x, obj_dir)
        back_threshold = -0.15
        return front_score >= back_threshold

    def make_grasp_pose(self, obj_P, reach_P, ee_pose):
        """Calculate the desired end-effector pose for grasping the object.
        The orientation of grasp is:
        - Z axis: points toward the object (normal vector from ee_pose to obj_pose)
        - X axis: parallel to XY plane, close to current EE X-axis to avoid large rotations
        - Y axis: cross product of Z and X
        Grasp position is equal to object position.
        careful when it go too close, the robot flip flop
        """
        ee_P = ee_pose[0:3, 3]
        ee_R = ee_pose[0:3, 0:3]

        # If object is behind EE, keep current orientation and only follow position.
        if not self.keep_grasp_orientation(
            obj_P=obj_P, reach_P=reach_P, ee_pose=ee_pose
        ):
            H = np.eye(4)
            H[0:3, 0:3] = ee_R
            H[0:3, 3] = reach_P
            return H

        diff = reach_P - ee_P
        diff_norm = np.linalg.norm(diff)
        if diff_norm < 0.1:  # if too close, keep current orientation to avoid flip
            H = np.eye(4)
            H[0:3, 0:3] = ee_R
            H[0:3, 3] = reach_P
            return H

        normz = diff / diff_norm

        # Get current EE X-axis and project to XY plane
        xdir_current = ee_pose[0:3, 0]
        xdir = np.array([xdir_current[0], xdir_current[1], 0.0])

        if np.linalg.norm(xdir) < 1e-12:
            # If current EE X-axis is vertical, use [1, 0, 0]
            xdir = np.array([1, 0, 0])
        else:
            xdir = xdir / np.linalg.norm(xdir)

        # Ensure xdir is perpendicular to normz using Gram-Schmidt
        xdir = xdir - np.dot(xdir, normz) * normz
        if np.linalg.norm(xdir) < 1e-12:
            # If xdir becomes zero after removing Z-component, use fallback
            xdir = np.cross([0, 0, 1], normz)
            if np.linalg.norm(xdir) < 1e-12:
                xdir = np.array([1, 0, 0])
            else:
                xdir = xdir / np.linalg.norm(xdir)
        else:
            xdir = xdir / np.linalg.norm(xdir)

        ydir = np.cross(normz, xdir)
        ydir = ydir / max(np.linalg.norm(ydir), 1e-12)

        # Keep orientation continuity: choose axis signs near current EE orientation.
        if np.dot(xdir, ee_R[:, 0]) < 0.0:
            xdir = -xdir
            ydir = -ydir

        # Rebuild orthonormal frame to avoid numeric drift.
        ydir = np.cross(normz, xdir)
        ydir = ydir / max(np.linalg.norm(ydir), 1e-12)
        xdir = np.cross(ydir, normz)
        xdir = xdir / max(np.linalg.norm(xdir), 1e-12)

        # Secondary continuity check on Y axis.
        if np.dot(ydir, ee_R[:, 1]) < 0.0:
            xdir = -xdir
            ydir = -ydir

        Rchase = np.column_stack((xdir, ydir, normz))

        H = np.eye(4)
        H[0:3, 0:3] = Rchase
        H[0:3, 3] = reach_P

        return H

    def offset_local_z(self, H, z_offset):
        """Apply a local Z offset to the given pose H."""
        R = H[0:3, 0:3]
        t = H[0:3, 3]
        t_offset = R @ np.array([0, 0, z_offset])
        H_offset = np.eye(4)
        H_offset[0:3, 0:3] = R
        H_offset[0:3, 3] = t + t_offset
        return H_offset

    def make_grasp_pose_fixedR(self, obj_P, reach_P, ee_pose, Rfixed):
        """Calculate the desired end-effector pose for grasping the object with fixed orientation."""
        H = np.eye(4)
        H[0:3, 0:3] = Rfixed
        # H[0:3, 3] = obj_P  # use object position as grasp position
        H[0:3, 3] = reach_P  # use reach position as grasp position
        return H

    def make_grasp_variable_z(self, obj_P, reach_P, ee_pose, Rfixed):
        Hgrasp = np.eye(4)
        Hgrasp[0:3, 3] = obj_P
        Hgrasp[:3, :3] = Rfixed
        # Hgrasp = Hgrasp @ self.make_h([0.0, 0.0, 0.02])
        Hgrasp = Hgrasp @ self.make_h([0.0, 0.0, -0.1])

        graspPointToBase = np.array([obj_P[0], obj_P[1], obj_P[2], 1.00])
        PpointTotcp = np.linalg.inv(ee_pose) @ graspPointToBase
        z = np.array([0.0, 1.0]).reshape(2, 1)
        p = np.array([PpointTotcp[0], PpointTotcp[2]]).reshape(2, 1)
        dotp = np.sum(z * p)
        normz = np.linalg.norm(z)
        normp = np.linalg.norm(p)
        alpha = np.arccos(dotp / (normz * normp))
        alphasign = alpha * np.sign(PpointTotcp[0])
        HorientCorrection = self.make_hy(alphasign)

        HgraspFinal = Hgrasp @ HorientCorrection
        return HgraspFinal

    def interp_q(self, q1, q2, alpha):
        """Interpolate between two joint configurations."""
        q1 = np.asarray(q1)
        q2 = np.asarray(q2)
        if np.linalg.norm(np.array(q1) - np.array(q2)) < 1e-12:
            return q2
        return (1 - alpha) * q1 + alpha * q2

    def make_h(self, t):
        H = np.eye(4)
        H[:3, 3] = t
        return H

    def make_hy(self, theta_rad):
        c = np.cos(theta_rad)
        s = np.sin(theta_rad)
        H = np.eye(4)
        H[0, 0] = c
        H[0, 2] = s
        H[2, 0] = -s
        H[2, 2] = c
        return H

    def make_retract(self, Hcurrent, retract_dist):
        """Make a retracted pose by moving along local -Z direction."""
        R = Hcurrent[0:3, 0:3]
        t = Hcurrent[0:3, 3]
        t_retract = R @ np.array([0, 0, -retract_dist])
        H_retract = np.eye(4)
        H_retract[0:3, 0:3] = R
        H_retract[0:3, 3] = t + t_retract
        return H_retract


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from matplotlib.patches import Ellipse
    from pytransform3d.plot_utils import make_3d_axis
    from pytransform3d.transform_manager import TransformManager
    from pytransform3d.transformations import plot_transform

    from aljnuho_v2.tracker_class import UKFRiskTracker
    from ur5e_ik import RobotUR5eKin, RobotController
    from msd import MSDRobotEE

    robot_real = RobotController()
    robot_kin = RobotUR5eKin()
    tracker = UKFRiskTracker()
    eemodel = MSDRobotEE()
    intsyst = IntegrateSystem()

    # live control parameters
    vel = 0.5
    acc = 0.5
    dt = 1.0 / 500  # 2ms
    lokat = 0.1
    gain = 300

    rot_step_deg = 5.0
    Hhome = robot_kin.solve_fk(intsyst.qhome)
    qcurrent = robot_real.get_actual_q()
    Hcurrent = robot_real.get_actual_tcp_pose()
    speed = np.zeros(3)
    speed6d = np.zeros(6)
    Rdesired = Hhome[0:3, 0:3]

    Hplace = np.array(
        [
            [0.0, -1.0, 0.0, 0.4],
            [-1.0, -0.0, -0.0, 0.4],
            [0.0, -0.0, -1.0, 0.3],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    # ax3d = make_3d_axis(ax_s=1.0)
    # (obj3d_line,) = ax3d.plot([], [], [], "ro", label="Grasp Pose")
    # robot_kin.plot_link_transforms(ax3d, qcurrent)
    # robot_kin.plot_parallel_gripper(ax3d, Hhome)

    # initial measurement
    obj_z_meas = np.array([0, 0, 0, 0, 0, 0])
    obj_z_meas_filtered = np.array([0, 0, 0, 0, 0, 0])

    # tracking current mouse position
    mouse_pos = {"x": None, "y": None, "z": 0.0}
    mouse_pos_prev = {"x": None, "y": None, "z": 0.0}
    real_action = {
        "move": False,
        "grip_delta": 0,
        "rot_x_delta": 0.0,
        "grip_pos": 0,
        "move_to_place": False,
        "obj_grasped": False,
        "move_chase_object": True,
        "move_to_home": False,
    }
    reach_mode = {"follow_ellipse": False}

    fig, ax2d = plt.subplots()
    ax2d.axhline(0, color="gray", linestyle="--")
    ax2d.axvline(0, color="gray", linestyle="--")
    ax2d.set_xlim(-1.0, 1.5)
    ax2d.set_ylim(-1.5, 1.5)
    ax2d.set_xlabel("X [m]")
    ax2d.set_ylabel("Z [m]")
    ax2d.set_aspect("equal")

    (ukf_line,) = ax2d.plot([], [], "bo", label="UKF Estimate")
    (measurement_line,) = ax2d.plot([], [], "ro", label="Measurement")
    (ee_current_line,) = ax2d.plot([], [], "gx", label="EE Current")
    (ee_obj_line,) = ax2d.plot([], [], "g-", linewidth=2, label="EE to Risk")
    (risk_proj_point,) = ax2d.plot([], [], "go", label="Risk Projection")
    (reach_proj_point,) = ax2d.plot([], [], "mo", label="Reach Projection")
    (centroid_line,) = ax2d.plot([], [], "rx", label="Centroid Measurement")

    el = Ellipse(
        (0, 0),
        width=1,
        height=1,
        edgecolor="red",
        facecolor="none",
        label="Covariance",
    )
    ax2d.add_patch(el)

    riskel = Ellipse(
        (0, 0),
        width=1,
        height=1,
        linestyle="--",
        edgecolor="orange",
        facecolor="none",
        label="Risk Ellipsoid",
    )
    ax2d.add_patch(riskel)
    creach = plt.Circle(
        (0, 0),
        intsyst.rreach,
        color="cyan",
        fill=False,
        linestyle="--",
        label="reaching radius",
    )
    ax2d.add_patch(creach)
    cworkspace = plt.Circle(
        (0, 0),
        intsyst.rworkspace,
        color="magenta",
        fill=False,
        linestyle="--",
        label="workspace boundary",
    )
    ax2d.add_patch(cworkspace)
    status_text = ax2d.text(
        0.02,
        0.98,
        "speed=0.00 | guard=HOLD",
        transform=ax2d.transAxes,
        ha="left",
        va="top",
    )
    status_text2 = ax2d.text(
        0.02,
        0.90,
        "",
        transform=ax2d.transAxes,
        ha="left",
        va="top",
    )

    def track_mouse(event):
        if event.xdata is not None and event.ydata is not None:
            mouse_pos["x"] = event.xdata
            mouse_pos["y"] = 0.0
            mouse_pos["z"] = event.ydata

    def toggle_action(event):
        global intsyst
        if event.key == "b":
            real_action["move"] = not real_action["move"]
        if event.key == "t":
            real_action["grip_delta"] += intsyst.gripper_step
        if event.key == "y":
            real_action["grip_delta"] -= intsyst.gripper_step
        if event.key == "u":
            real_action["rot_x_delta"] += np.deg2rad(rot_step_deg)
        if event.key == "i":
            real_action["rot_x_delta"] -= np.deg2rad(rot_step_deg)
        if event.key == "p":
            real_action["move_to_place"] = False
        if event.key == "o":
            real_action["move_to_place"] = True
        if event.key == "c":
            real_action["move_chase_object"] = not real_action["move_chase_object"]
        if event.key == "h":
            real_action["move_to_home"] = not real_action["move_to_home"]
        if event.key == "g":
            real_action["obj_grasped"] = not real_action["obj_grasped"]

    # set real to initial state
    robot_real.move_joints(intsyst.qhome)
    robot_real.move_gripper(real_action["grip_pos"])

    # shm = shared_memory.SharedMemory(name="psm_bb2c88e9")
    # centroids = np.ndarray((4,), dtype=np.float64, buffer=shm.buf)

    def loop():
        global obj_z_meas, obj_z_meas_filtered, Hhome, speed, speed6d, Rdesired
        t_start = robot_real.rtde_c.initPeriod()

        if mouse_pos["x"] is not None and mouse_pos["z"] is not None:

            xc = mouse_pos["x"]
            yc = mouse_pos["y"]
            zc = mouse_pos["z"]

            # Calculate velocity from previous measurement
            if mouse_pos_prev["x"] is not None:
                vxc = (xc - obj_z_meas[0]) / tracker.dt
                vyc = (yc - obj_z_meas[1]) / tracker.dt
                vzc = (zc - obj_z_meas[2]) / tracker.dt
            else:
                vxc = 0
                vyc = 0
                vzc = 0

            # Update measurement with velocity
            obj_z_meas[:] = [xc, yc, zc, vxc, vyc, vzc]
            obj_z_meas = obj_z_meas + 0.01 * np.random.multivariate_normal(
                np.zeros(6), tracker.ukf.R
            )

            # Apply low pass filter to measurement
            obj_z_meas_filtered[:] = (
                tracker.lpf_alpha * obj_z_meas
                + (1 - tracker.lpf_alpha) * obj_z_meas_filtered
            )

            # UKF predict and update with filtered measurement
            tracker.ukf.predict()
            tracker.ukf.update(obj_z_meas)

            # Update plot
            xobj = tracker.ukf.x[0]
            yobj = tracker.ukf.x[1]
            zobj = tracker.ukf.x[2]
            ukf_line.set_data([xobj], [zobj])
            measurement_line.set_data([obj_z_meas[0]], [obj_z_meas[2]])

            # update covariance ellipse
            cov_e_w, cov_e_h, cov_e_d, cov_e_angle_xy, cov_e_angle_xz = (
                tracker.pose_covariance_ellipse_3d(tracker.ukf.P)
            )
            el.center = (tracker.ukf.x[0], tracker.ukf.x[2])
            el.width = cov_e_w
            el.height = cov_e_d
            el.angle = cov_e_angle_xz

            # update risk ellipse
            speed_xyz = np.linalg.norm(tracker.ukf.x[3:6])
            rk_e_w, rk_e_h, rk_e_d, rk_e_angle_xy, rk_e_angle_xz = (
                tracker.risk_covariance_ellipse_3d(tracker.ukf.P, speed=speed_xyz)
            )
            riskel.center = (tracker.ukf.x[0], tracker.ukf.x[2])
            riskel.width = rk_e_w
            riskel.height = rk_e_d
            riskel.angle = rk_e_angle_xz

            guard_action, guard_color = tracker.guard_action_from_speed(speed_xyz)
            riskel.set_edgecolor(guard_color)
            status_text.set_text(f"speed={speed_xyz:.2f} | guard={guard_action}")
            status_text.set_color(guard_color)

            # detect if object is out of workspace boundary
            if not intsyst.is_obj_in_boundary(
                (xobj, yobj, zobj), intsyst.rworkspace
            ):
                radius_bound = intsyst.rreach
                status_text2.set_text("Object out of workspace boundary!")
                status_text2.set_color("red")
            else:
                status_text2.set_text("Object within workspace boundary.")
                status_text2.set_color("black")
                radius_bound = intsyst.rworkspace

            # grasp point projection and reach projection
            xproj, yproj, zproj = intsyst.project_point_on_rotated_ellipse3d(
                center=(xobj, yobj, zobj),
                width=rk_e_w,
                height=rk_e_h,
                depth=rk_e_d,
                angle_xy_deg=rk_e_angle_xy,
                angle_xz_deg=rk_e_angle_xz,
                target_point=(Hhome[0, 3], Hhome[1, 3], Hhome[2, 3]),
            )
            xreach, yreach, zreach, reach_mode["follow_ellipse"] = (
                intsyst.select_reach_target_model_3d(
                    ee_point=(Hhome[0, 3], Hhome[1, 3], Hhome[2, 3]),
                    ellipse_proj_point=(xproj, yproj, zproj),
                    radius=radius_bound,
                    follow_ellipse=reach_mode["follow_ellipse"],
                )
            )

            # obj3d_line.set_data_3d([xc], [0.0], [yc])
            ee_obj_line.set_data([Hhome[0, 3], xproj], [Hhome[2, 3], zproj])
            risk_proj_point.set_data([xproj], [zproj])
            reach_proj_point.set_data([xreach], [zreach])
            # centroid_line.set_data([centroids[0]], [centroids[2]])

            if real_action["move"]:
                # add msd model to ee and xreach point
                Hcurrent = robot_real.get_actual_tcp_pose()
                ee_current_line.set_data([Hcurrent[0, 3]], [Hcurrent[2, 3]])
                # poses_c = Hcurrent[0:3, 3]

                if real_action["obj_grasped"]:
                    if real_action["move_to_place"]:
                        Hctrl, speed = eemodel.step2(
                            T_ee=Hcurrent,
                            vel3d=speed,
                            T_obj=Hplace,
                        )
                        pose_cl = robot_real.make_pose_from_H(Hctrl)
                    else:
                        pass  # already grasped and not moving to place, do nothing
                        # hold current pose
                        pose_cl = robot_real.make_pose_from_H(Hcurrent)

                # if not grasped, control to follow reach point
                elif not real_action["obj_grasped"]:
                    if real_action["move_chase_object"]:
                        obj_pos = np.array([xobj, yobj, zobj])
                        reach_pos = np.array([xreach, yreach, zreach])

                        Hdesired = intsyst.make_grasp_pose(
                            obj_P=obj_pos,
                            reach_P=reach_pos,
                            ee_pose=Hcurrent,
                        )

                        Hctrl, speed = eemodel.step2(
                            T_ee=Hcurrent,
                            vel3d=speed,
                            T_obj=Hdesired,
                        )
                        pose_cl = robot_real.make_pose_from_H(Hctrl)

                elif real_action["move_to_home"]:
                    Hctrl = Hhome
                    pose_cl = robot_real.make_pose_from_H(Hctrl)

                # control robot to follow the projected point on reach circle
                robot_real.rtde_c.servoL(pose_cl, vel, acc, dt, lokat, gain)
                robot_real.rtde_c.waitPeriod(t_start)

            if real_action["grip_delta"] != 0:
                next_pos = real_action["grip_pos"] + real_action["grip_delta"]
                real_action["grip_pos"] = next_pos
                robot_real.move_gripper(real_action["grip_pos"])
                # Consume pending keypress commands once to avoid continuous drift.
                real_action["grip_delta"] = 0

            # draw
            fig.canvas.draw_idle()
            # ax3d.figure.canvas.draw_idle()

            # Update previous position
            mouse_pos_prev["x"] = mouse_pos["x"]
            mouse_pos_prev["y"] = mouse_pos["y"]
            mouse_pos_prev["z"] = mouse_pos["z"]

    ax2d.legend(loc="upper right", bbox_to_anchor=(1.15, 1))
    fig.canvas.mpl_connect("motion_notify_event", track_mouse)
    fig.canvas.mpl_connect("key_press_event", toggle_action)
    timer = fig.canvas.new_timer(interval=int(tracker.dt * 1000))
    timer.add_callback(loop)
    timer.start()
    plt.show()

    robot_real.rtde_c.servoStop()
    robot_real.rtde_c.stopScript()
    print("Program terminated.")
