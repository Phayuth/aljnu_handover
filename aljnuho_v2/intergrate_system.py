import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from aljnuho_v2.tracker_class import UKFRiskTracker
from ur5e_ik import RobotUR5eKin, RobotController
from msd import MSDRobotEE
import threading
import cv2
from pytransform3d.plot_utils import make_3d_axis
from pytransform3d.transform_manager import TransformManager
from pytransform3d.transformations import plot_transform


class TrajectoryOptimizer:

    def __init__(self):
        self.x_start = np.array([0.0, 0.0])


class IntegrateSystem:

    def __init__(self):
        pass


def project_point_on_rotated_ellipse(
    center, width, height, angle_deg, target_point
):
    """Project from ellipse center toward target_point onto rotated ellipse boundary."""
    cx, cz = center
    tx, tz = target_point

    a = max(width * 0.5, 1e-9)
    b = max(height * 0.5, 1e-9)

    v_world = np.array([tx - cx, tz - cz], dtype=float)
    if np.linalg.norm(v_world) < 1e-12:
        return cx, cz

    theta = np.deg2rad(angle_deg)
    cth = np.cos(theta)
    sth = np.sin(theta)

    # Rotate world vector into ellipse local frame.
    v_local_x = cth * v_world[0] + sth * v_world[1]
    v_local_z = -sth * v_world[0] + cth * v_world[1]

    denom = np.sqrt((v_local_x / a) ** 2 + (v_local_z / b) ** 2)
    if denom < 1e-12:
        return cx, cz

    p_local = np.array([v_local_x / denom, v_local_z / denom])

    # Rotate boundary point back to world frame.
    px = cth * p_local[0] - sth * p_local[1] + cx
    pz = sth * p_local[0] + cth * p_local[1] + cz
    return px, pz


def project_line_to_reach_circle(ee_point, toward_point, radius):
    """Intersect ray from ee_point toward toward_point with circle centered at origin."""
    ex, ez = ee_point
    tx, tz = toward_point
    d = np.array([tx - ex, tz - ez], dtype=float)
    if np.linalg.norm(d) < 1e-12:
        return ex, ez

    p = np.array([ex, ez], dtype=float)
    a = np.dot(d, d)
    b = 2.0 * np.dot(p, d)
    c = np.dot(p, p) - radius**2
    disc = b * b - 4.0 * a * c

    if disc < 0.0:
        return ex, ez

    sqrt_disc = np.sqrt(disc)
    t1 = (-b - sqrt_disc) / (2.0 * a)
    t2 = (-b + sqrt_disc) / (2.0 * a)
    t_candidates = [t for t in (t1, t2) if t >= 0.0]
    if not t_candidates:
        return ex, ez

    t = max(t_candidates)
    p_hit = p + t * d
    return p_hit[0], p_hit[1]


def select_reach_target_model(
    ee_point,
    ellipse_proj_point,
    radius,
    follow_ellipse,
    hysteresis=0.02,
):
    """Select target on reach circle or ellipse projection using a mode switch.

    Rule:
    - Stay on reach circle by default.
    - Switch to ellipse projection when ellipse projection enters reach radius.
    - Use small hysteresis to avoid mode chattering near boundary.
    """
    proj_r = np.hypot(ellipse_proj_point[0], ellipse_proj_point[1])

    enter = proj_r <= radius
    leave = proj_r > radius + hysteresis

    if follow_ellipse:
        if leave:
            follow_ellipse = False
    else:
        if enter:
            follow_ellipse = True

    if follow_ellipse:
        return ellipse_proj_point[0], ellipse_proj_point[1], follow_ellipse

    xreach, zreach = project_line_to_reach_circle(
        ee_point=ee_point,
        toward_point=ellipse_proj_point,
        radius=radius,
    )
    return xreach, zreach, follow_ellipse


def rot_x(theta_rad):
    """Rotation matrix about +X axis."""
    c = np.cos(theta_rad)
    s = np.sin(theta_rad)
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ]
    )


if __name__ == "__main__":
    robot_real = RobotController()
    robot_kin = RobotUR5eKin()
    tracker = UKFRiskTracker()
    system = IntegrateSystem()
    topt = TrajectoryOptimizer()
    eemodel = MSDRobotEE()

    rreach = 0.85

    # live control parameters
    vel = 0.5
    acc = 0.5
    dt = 1.0 / 500  # 2ms
    lookahead_time = 0.1
    gain = 300

    qhome = [3.14, -1.610, 1.61, -1.558, -1.562, 0.0]
    gripper_step = 25
    gripper_min = 0
    gripper_max = 255
    rot_step_deg = 5.0
    Hhome = robot_kin.solve_fk(qhome)
    qcurrent = robot_real.get_actual_q()
    Hcurrent = robot_real.get_actual_tcp_pose()
    speed = np.zeros(3)
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

    # (grasp_line,) = ax2d.plot([], [], "g-", label="Grasp Approach")
    # (traj_line,) = ax2d.plot([], [], "m-", label="Optimized Trajectory")

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
        rreach,
        color="cyan",
        fill=False,
        linestyle="--",
        label="reaching radius",
    )
    ax2d.add_patch(creach)
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
        "Action = False",
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
        if event.key == "b":
            real_action["move"] = not real_action["move"]
        if event.key == "t":
            real_action["grip_delta"] += gripper_step
        if event.key == "y":
            real_action["grip_delta"] -= gripper_step
        if event.key == "u":
            real_action["rot_x_delta"] += np.deg2rad(rot_step_deg)
        if event.key == "i":
            real_action["rot_x_delta"] -= np.deg2rad(rot_step_deg)
        if event.key == "p":
            real_action["move_to_place"] = True

    # set real to initial state
    robot_real.move_joints(qhome)
    robot_real.move_gripper(real_action["grip_pos"])

    def loop():
        global obj_z_meas, obj_z_meas_filtered, Hhome, speed
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
            yobj = 0.0
            zobj = tracker.ukf.x[2]
            ukf_line.set_data([xobj], [zobj])
            measurement_line.set_data([obj_z_meas[0]], [obj_z_meas[2]])

            # update covariance ellipse
            el.center = (tracker.ukf.x[0], tracker.ukf.x[2])
            el.width, el.height, el.angle = tracker.pose_covariance_ellipse(
                tracker.ukf.P
            )

            speed_xy = np.linalg.norm(tracker.ukf.x[3:5])
            guard_action, guard_color = tracker.guard_action_from_speed(speed_xy)
            riskel.center = (tracker.ukf.x[0], tracker.ukf.x[2])
            riskel.width, riskel.height, riskel.angle = (
                tracker.risk_covariance_ellipse(tracker.ukf.P, speed=speed_xy)
            )
            riskel.set_edgecolor(guard_color)
            status_text.set_text(f"speed={speed_xy:.2f} | guard={guard_action}")
            status_text.set_color(guard_color)

            xproj, zproj = project_point_on_rotated_ellipse(
                center=(xobj, zobj),
                width=riskel.width,
                height=riskel.height,
                angle_deg=riskel.angle,
                target_point=(Hhome[0, 3], Hhome[2, 3]),
            )

            xreach, zreach, reach_mode["follow_ellipse"] = (
                select_reach_target_model(
                    ee_point=(Hhome[0, 3], Hhome[2, 3]),
                    ellipse_proj_point=(xproj, zproj),
                    radius=rreach,
                    follow_ellipse=reach_mode["follow_ellipse"],
                )
            )

            # obj3d_line.set_data_3d([xc], [0.0], [yc])
            ee_obj_line.set_data([Hhome[0, 3], xproj], [Hhome[2, 3], zproj])
            risk_proj_point.set_data([xproj], [zproj])
            reach_proj_point.set_data([xreach], [zreach])
            status_text2.set_text(f"Action = {real_action['move']}")

            if real_action["rot_x_delta"] != 0.0:
                # Local-frame rotation: post-multiply so +X is TCP's own axis.
                Rdesired[:, :] = Rdesired @ rot_x(real_action["rot_x_delta"])
                # Keep a proper orthonormal rotation matrix after repeated steps.
                u, _, vt = np.linalg.svd(Rdesired)
                Rdesired[:, :] = u @ vt
                real_action["rot_x_delta"] = 0.0

            if real_action["move"]:
                # add msd model to ee and xreach point
                Hcurrent = robot_real.get_actual_tcp_pose()
                poses_c = Hcurrent[0:3, 3]
                object_pos = np.array([xreach, 0.0, zreach])
                pos_ctrl, speed = eemodel.step(
                    poses_c,
                    speed,
                    object_pos,
                )
                ee_current_line.set_data([Hcurrent[0, 3]], [Hcurrent[2, 3]])

                Hctrl = np.eye(4)
                Hctrl[0:3, 0:3] = Rdesired
                Hctrl[0, 3] = pos_ctrl[0]
                Hctrl[2, 3] = pos_ctrl[2]
                pose = robot_real.make_pose_from_H(Hctrl)
                robot_real.rtde_c.servoL(pose, vel, acc, dt, lookahead_time, gain)
                robot_real.rtde_c.waitPeriod(t_start)

            if real_action["grip_delta"] != 0:
                next_pos = real_action["grip_pos"] + real_action["grip_delta"]
                real_action["grip_pos"] = int(
                    np.clip(next_pos, gripper_min, gripper_max)
                )
                robot_real.move_gripper(real_action["grip_pos"])
                # Consume pending keypress commands once to avoid continuous drift.
                real_action["grip_delta"] = 0

            if real_action["move_to_place"]:
                place_pose = robot_real.make_pose_from_H(Hplace)
                robot_real.rtde_c.servoL(place_pose, 0.01, 0.01, dt, lookahead_time, gain)
                real_action["move_to_place"] = False

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
