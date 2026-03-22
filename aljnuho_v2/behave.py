import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints

dt = 0.1


# ---------------- UKF ----------------
def fx(x, dt):
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


def hx(x):
    return x


points = MerweScaledSigmaPoints(n=6, alpha=0.1, beta=2.0, kappa=0)
ukf = UKF(dim_x=6, dim_z=6, fx=fx, hx=hx, dt=dt, points=points)

ukf.x = np.zeros(6)
ukf.P *= 0.1
ukf.Q = np.eye(6) * 1e-4
ukf.R = np.eye(6) * 5e-3  # moderate noise


# ---------------- helpers ----------------
def get_ellipsoid(P, center, scale=np.sqrt(7.815)):
    P_pos = P[:3, :3]
    w, v = np.linalg.eigh(P_pos)
    axes = scale * np.sqrt(np.maximum(w, 1e-12))

    u = np.linspace(0, 2 * np.pi, 20)
    v_ang = np.linspace(0, np.pi, 10)

    x = axes[0] * np.outer(np.cos(u), np.sin(v_ang))
    y = axes[1] * np.outer(np.sin(u), np.sin(v_ang))
    z = axes[2] * np.outer(np.ones_like(u), np.cos(v_ang))

    ell = np.stack((x, y, z), axis=-1)
    ell = ell @ v.T + center
    return ell


def ellipsoid_surface_target(mu, P, robot_pos):
    P_pos = P[:3, :3]
    d = mu - robot_pos

    invP = np.linalg.inv(P_pos + 1e-9 * np.eye(3))
    chi2 = 7.815

    denom = d.T @ invP @ d
    if denom < 1e-9:
        return mu

    lam = np.sqrt(chi2 / denom)
    return mu - lam * d


# ---------------- simulation ----------------
N = 200

true_states = []
estimates = []
Ps = []
robot_traj = []
targets = []
Peff = []

# robot initial position
robot_pos = np.array([-2.0, -2.0, 0.0])

r = 1.0
omega = 0.8
vz = 0.2

for k in range(N):
    t = k * dt

    # ---- ground truth (spiral)
    x = r * np.cos(omega * t)
    y = r * np.sin(omega * t)
    z = vz * t
    vx = -r * omega * np.sin(omega * t)
    vy = r * omega * np.cos(omega * t)
    if t > 10:
        x = true_states[-1][0]
        y = true_states[-1][1]
        z = true_states[-1][2]
        vx = 0.0
        vy = 0.0
        vz = 0.0  # stop ascending after some time
    x_true = np.array([x, y, z, vx, vy, vz])

    # measurement
    z_meas = x_true + np.random.multivariate_normal(np.zeros(6), ukf.R)

    # UKF
    ukf.predict()
    ukf.update(z_meas)

    mu = ukf.x[:3]
    v_est = ukf.x[3:]
    P = ukf.P.copy()

    # ---- behavior logic
    pos_unc = np.trace(P[:3, :3])
    vel_mag = np.linalg.norm(v_est)

    if pos_unc > 0.05 or vel_mag > 0.8:
        mode_scale = 1.0  # cautious (stay on ellipse)
    elif pos_unc > 0.01:
        mode_scale = 0.5  # approach
    else:
        mode_scale = 0.1  # grasp

    # inflate/deflate covariance
    P_eff = P.copy()
    P_eff[:3, :3] *= mode_scale

    target = ellipsoid_surface_target(mu, P_eff, robot_pos)

    # simple robot motion (first-order)
    robot_pos = robot_pos + 0.1 * (target - robot_pos)

    # store
    true_states.append(x_true)
    estimates.append(ukf.x.copy())
    Ps.append(P)
    robot_traj.append(robot_pos.copy())
    targets.append(target.copy())
    Peff.append(P_eff)

true_states = np.array(true_states)
estimates = np.array(estimates)
robot_traj = np.array(robot_traj)
targets = np.array(targets)
Peff = np.array(Peff)

# ---------------- animation ----------------
fig = plt.figure()
ax = fig.add_subplot(projection="3d")

(true_line,) = ax.plot([], [], [], label="true")
(est_line,) = ax.plot([], [], [], label="ukf")
(robot_line,) = ax.plot([], [], [], label="robot")
(target_point,) = ax.plot([], [], [], "o", label="target")
(target_line,) = ax.plot([], [], [], linestyle="--", alpha=0.5)
ell_plot = [None]
ell_eff_plot = [None]

ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(0, np.max(true_states[:, 2]) + 0.5)

ax.legend()


def update(frame):
    true_line.set_data(true_states[:frame, 0], true_states[:frame, 1])
    true_line.set_3d_properties(true_states[:frame, 2])

    est_line.set_data(estimates[:frame, 0], estimates[:frame, 1])
    est_line.set_3d_properties(estimates[:frame, 2])

    robot_line.set_data(robot_traj[:frame, 0], robot_traj[:frame, 1])
    robot_line.set_3d_properties(robot_traj[:frame, 2])
    # target trail
    target_line.set_data(targets[:frame, 0], targets[:frame, 1])
    target_line.set_3d_properties(targets[:frame, 2])

    # current target point
    target_point.set_data([targets[frame, 0]], [targets[frame, 1]])
    target_point.set_3d_properties([targets[frame, 2]])

    # ellipsoid
    if ell_plot[0] is not None:
        ell_plot[0].remove()

    ell = get_ellipsoid(Ps[frame], estimates[frame, :3])
    ell_plot[0] = ax.plot_wireframe(
        ell[:, :, 0], ell[:, :, 1], ell[:, :, 2], alpha=0.2
    )

    # effective ellipsoid
    if ell_eff_plot[0] is not None:
        ell_eff_plot[0].remove()

    ell_eff = get_ellipsoid(Peff[frame], estimates[frame, :3])
    ell_eff_plot[0] = ax.plot_wireframe(
        ell_eff[:, :, 0], ell_eff[:, :, 1], ell_eff[:, :, 2], color="r", alpha=0.2
    )

    return true_line, est_line, robot_line


ani = FuncAnimation(fig, update, frames=N, interval=500)

plt.show()
