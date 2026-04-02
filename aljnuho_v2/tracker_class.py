import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints


class UKFRiskTracker:

    def __init__(self):
        # UKF parameters
        self.dt = 0.01
        self.lpf_alpha = 0.9  # Low pass filter coefficient

        points = MerweScaledSigmaPoints(n=6, alpha=0.1, beta=2.0, kappa=0)
        self.ukf = UKF(
            dim_x=6,
            dim_z=6,
            fx=self.fx,
            hx=self.hx,
            dt=self.dt,
            points=points,
        )

        # initial state: [x, y, z, vx, vy, vz]
        self.ukf.x = np.array([0, 0, 0, 0, 0, 0])
        self.ukf.P *= 0.1
        self.ukf.Q = np.eye(6) * 1e-4
        self.ukf.R = np.eye(6) * 1e-2

        # initial measurement
        self.z_meas = np.array([0, 0, 0, 0, 0, 0])
        self.z_meas_filtered = np.array([0, 0, 0, 0, 0, 0])

        # Risk model tuning
        self.risk_n_std = 2.0
        self.risk_vel_gain = 1.0
        self.risk_scale_min = 1.0
        self.risk_scale_max = 8.0
        self.retreat_speed_th = 1.0
        self.approach_speed_th = 0.25

    def fx(self, x, dt):
        F = np.array(
            [
                [1, 0, 0, dt, 0, 0],
                [0, 1, 0, 0, dt, 0],
                [0, 0, 1, 0, 0, dt],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ]
        )
        return F @ x

    def hx(self, x):
        return x

    def pose_covariance_ellipse(self, P, n_std=2.0):
        """Return ellipse width, height, and angle from a 2x2 covariance matrix."""
        eigvals, eigvecs = np.linalg.eigh(P[:2, :2])
        order = np.argsort(eigvals)[::-1]
        eigvals = eigvals[order]
        eigvecs = eigvecs[:, order]

        width = 2.0 * n_std * np.sqrt(max(eigvals[0], 1e-12))
        height = 2.0 * n_std * np.sqrt(max(eigvals[1], 1e-12))
        angle = np.degrees(np.arctan2(eigvecs[1, 0], eigvecs[0, 0]))
        return width, height, angle

    def speed_to_risk_scale(self, speed):
        scale = 1.0 + self.risk_vel_gain * speed
        return np.clip(scale, self.risk_scale_min, self.risk_scale_max)

    def guard_action_from_speed(self, speed):
        if speed >= self.retreat_speed_th:
            return "MOVE BACK", "red"
        if speed <= self.approach_speed_th:
            return "MOVE IN", "green"
        return "HOLD", "orange"

    def risk_covariance_ellipse(self, P, speed):
        """Return risk ellipse (width, height, angle) with velocity-based scaling."""
        width, height, angle = self.pose_covariance_ellipse(
            P, n_std=self.risk_n_std
        )
        scale = self.speed_to_risk_scale(speed)
        return width * scale, height * scale, angle


if __name__ == "__main__":
    tracker = UKFRiskTracker()

    # initial measurement
    z_meas = np.array([0, 0, 0, 0, 0, 0])
    z_meas_filtered = np.array([0, 0, 0, 0, 0, 0])

    # tracking current mouse position
    mouse_pos = {"x": None, "y": None}
    mouse_pos_prev = {"x": None, "y": None}

    fig, ax = plt.subplots()
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_aspect("equal")
    (ukf_line,) = ax.plot([], [], "bo", label="UKF Estimate")
    (measurement_line,) = ax.plot([], [], "ro", label="Measurement")

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

    status_text = ax.text(
        0.02,
        0.98,
        "speed=0.00 | guard=HOLD",
        transform=ax.transAxes,
        ha="left",
        va="top",
    )

    ax.legend()

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
            fig.canvas.draw_idle()

            # Update previous position
            mouse_pos_prev["x"] = mouse_pos["x"]
            mouse_pos_prev["y"] = mouse_pos["y"]

    fig.canvas.mpl_connect("motion_notify_event", track_mouse)
    timer = fig.canvas.new_timer(interval=int(tracker.dt * 1000))
    timer.add_callback(loop)
    timer.start()
    plt.show()
