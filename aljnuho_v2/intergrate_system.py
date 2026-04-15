import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from aljnuho_v2.tracker_class import UKFRiskTracker


class GraspApproachCone:

    def __init__(self):
        self.cone_length = 0.5
        self.cone_radius = 0.3
        self.r_min = 0.0
        self.r_max = self.cone_length

    def R(self, th):
        return np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])

    def get_grasp_approach(self, target_pos):
        angle = -1.0
        grasp_pose = np.array([target_pos[0], target_pos[1], angle])
        return grasp_pose

    def get_cone(self, target_grasp):
        target_pos = target_grasp[:2]
        n = np.array([np.cos(target_grasp[2]), np.sin(target_grasp[2])])
        n = n / np.linalg.norm(n)

        t = np.array([self.r_min, self.r_max])
        v1 = self.R(self.cone_radius) @ n
        v2 = self.R(-self.cone_radius) @ n
        ray1 = target_pos.reshape(2, 1) - v1.reshape(2, 1) * t
        ray2 = target_pos.reshape(2, 1) - v2.reshape(2, 1) * t

        return ray1, ray2


class TrajectoryOptimizer:

    def __init__(self):
        self.x_start = np.array([0.0, 0.0])


class IntegrateSystem:

    def __int__(self):
        pass


if __name__ == "__main__":
    tracker = UKFRiskTracker()
    grasp = GraspApproachCone()
    system = IntegrateSystem()
    topt = TrajectoryOptimizer()
    rreach = 0.85
    # initial measurement
    z_meas = np.array([0, 0, 0, 0, 0, 0])
    z_meas_filtered = np.array([0, 0, 0, 0, 0, 0])

    # tracking current mouse position
    mouse_pos = {"x": None, "y": None}
    mouse_pos_prev = {"x": None, "y": None}

    fig, ax = plt.subplots()
    ax.axhline(0, color="gray", linestyle="--")
    ax.axvline(0, color="gray", linestyle="--")
    ax.set_xlim(-1.0, 3.0)
    ax.set_ylim(-2.0, 2.0)
    ax.set_aspect("equal")

    (ukf_line,) = ax.plot([], [], "bo", label="UKF Estimate")
    (measurement_line,) = ax.plot([], [], "ro", label="Measurement")
    (grasp_line,) = ax.plot([], [], "g-", label="Grasp Approach")
    (ray1_line,) = ax.plot([], [], "c--", label="Cone Ray 1")
    (ray2_line,) = ax.plot([], [], "c--", label="Cone Ray 2")
    (traj_line,) = ax.plot([], [], "m-", label="Optimized Trajectory")

    el = Ellipse(
        (0, 0),
        width=1,
        height=1,
        edgecolor="red",
        facecolor="none",
        label="Covariance",
    )
    ax.add_patch(el)

    riskel = Ellipse(
        (0, 0),
        width=1,
        height=1,
        linestyle="--",
        edgecolor="orange",
        facecolor="none",
        label="Risk Ellipsoid",
    )
    ax.add_patch(riskel)
    creach = plt.Circle(
        (0, 0),
        rreach,
        color="cyan",
        fill=False,
        linestyle="--",
        label="reaching radius",
    )
    ax.add_patch(creach)
    status_text = ax.text(
        0.02,
        0.98,
        "speed=0.00 | guard=HOLD",
        transform=ax.transAxes,
        ha="left",
        va="top",
    )

    def track_mouse(event):
        if event.xdata is not None and event.ydata is not None:
            mouse_pos["x"] = event.xdata
            mouse_pos["y"] = event.ydata

    def loop():
        global z_meas, z_meas_filtered

        if mouse_pos["x"] is not None and mouse_pos["y"] is not None:

            xc = mouse_pos["x"]
            yc = mouse_pos["y"]

            # Calculate velocity from previous measurement
            if mouse_pos_prev["x"] is not None:
                vxc = (xc - z_meas[0]) / tracker.dt
                vyc = (yc - z_meas[1]) / tracker.dt
            else:
                vxc = 0
                vyc = 0

            # Update measurement with velocity
            z_meas[:] = [xc, yc, 0, vxc, vyc, 0]
            z_meas = z_meas + 0.01 * np.random.multivariate_normal(
                np.zeros(6), tracker.ukf.R
            )

            # Apply low pass filter to measurement
            z_meas_filtered[:] = (
                tracker.lpf_alpha * z_meas
                + (1 - tracker.lpf_alpha) * z_meas_filtered
            )

            # UKF predict and update with filtered measurement
            tracker.ukf.predict()
            tracker.ukf.update(z_meas)

            # Update plot
            ukf_line.set_data([tracker.ukf.x[0]], [tracker.ukf.x[1]])
            measurement_line.set_data([z_meas[0]], [z_meas[1]])

            # update covariance ellipse
            el.center = (tracker.ukf.x[0], tracker.ukf.x[1])
            el.width, el.height, el.angle = tracker.pose_covariance_ellipse(
                tracker.ukf.P
            )

            speed_xy = np.linalg.norm(tracker.ukf.x[3:5])
            guard_action, guard_color = tracker.guard_action_from_speed(speed_xy)
            riskel.center = (tracker.ukf.x[0], tracker.ukf.x[1])
            riskel.width, riskel.height, riskel.angle = (
                tracker.risk_covariance_ellipse(tracker.ukf.P, speed=speed_xy)
            )
            riskel.set_edgecolor(guard_color)
            status_text.set_text(f"speed={speed_xy:.2f} | guard={guard_action}")
            status_text.set_color(guard_color)

            # grasp
            grasp_pose = grasp.get_grasp_approach(tracker.ukf.x[:2])
            grasp_line.set_data(
                [tracker.ukf.x[0], tracker.ukf.x[0] + 0.5 * np.cos(grasp_pose[2])],
                [tracker.ukf.x[1], tracker.ukf.x[1] + 0.5 * np.sin(grasp_pose[2])],
            )
            ray1, ray2 = grasp.get_cone(grasp_pose)
            ray1_line.set_data(ray1[0], ray1[1])
            ray2_line.set_data(ray2[0], ray2[1])

            fig.canvas.draw_idle()

            # Update previous position
            mouse_pos_prev["x"] = mouse_pos["x"]
            mouse_pos_prev["y"] = mouse_pos["y"]

    ax.legend(loc="upper right", bbox_to_anchor=(1.15, 1))
    fig.canvas.mpl_connect("motion_notify_event", track_mouse)
    timer = fig.canvas.new_timer(interval=int(tracker.dt * 1000))
    timer.add_callback(loop)
    timer.start()
    plt.show()
