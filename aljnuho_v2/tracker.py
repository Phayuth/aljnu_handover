import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
from pytransform3d.transformations import plot_transform

dt = 0.1


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

# initial state: [x, y, z, vx, vy, vz]
ukf.x = np.array([0, 0, 0, 0, 0, 0])
ukf.P *= 0.1
ukf.Q = np.eye(6) * 1e-4
ukf.R = np.eye(6) * 1e-2

# simulate
N = 200
true_states, estimates = [], []
Ps = []
r = 1.0
omega = 0.8  # angular speed
vz = 0.2  # upward velocity

for k in range(N):
    t = k * dt

    # true measurement state: spiral position, velocity
    x = r * np.cos(omega * t)
    y = r * np.sin(omega * t)
    z = vz * t
    vx = -r * omega * np.sin(omega * t)
    vy = r * omega * np.cos(omega * t)
    x_true = np.array([x, y, z, vx, vy, vz])

    z_meas = x_true + np.random.multivariate_normal(np.zeros(6), ukf.R)

    ukf.predict()
    ukf.update(z_meas)

    true_states.append(x_true.copy())
    estimates.append(ukf.x.copy())
    Ps.append(ukf.P.copy())

true_states = np.array(true_states)
estimates = np.array(estimates)

# ---- animation
fig = plt.figure()
ax = fig.add_subplot(projection="3d")
plot_transform(ax, np.eye(4), s=0.5)  # plot world frame

(true_line,) = ax.plot([], [], [], label="true")
(est_line,) = ax.plot([], [], [], label="ukf")

ax.set_xlim(np.min(true_states[:, 0]), np.max(true_states[:, 0]))
ax.set_ylim(np.min(true_states[:, 1]), np.max(true_states[:, 1]))
ax.set_zlim(np.min(true_states[:, 2]), np.max(true_states[:, 2]))
ax.set_box_aspect([1.0, 1.0, 1.0])
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.legend()


def get_ellipsoid(P, center, scale=np.sqrt(7.815)):
    P_pos = P[:3, :3]
    w, v = np.linalg.eigh(P_pos)

    axes = scale * np.sqrt(np.maximum(w, 1e-12))  # numerical safety

    u = np.linspace(0, 2 * np.pi, 20)
    v_ang = np.linspace(0, np.pi, 10)

    x = axes[0] * np.outer(np.cos(u), np.sin(v_ang))
    y = axes[1] * np.outer(np.sin(u), np.sin(v_ang))
    z = axes[2] * np.outer(np.ones_like(u), np.cos(v_ang))

    ell = np.stack((x, y, z), axis=-1)
    ell = ell @ v.T + center

    return ell


ell_plot = [None]


def update(frame):
    true_line.set_data(true_states[:frame, 0], true_states[:frame, 1])
    true_line.set_3d_properties(true_states[:frame, 2])

    est_line.set_data(estimates[:frame, 0], estimates[:frame, 1])
    est_line.set_3d_properties(estimates[:frame, 2])
    # remove previous ellipsoid
    if ell_plot[0] is not None:
        ell_plot[0].remove()

    # compute new ellipsoid
    ell = get_ellipsoid(Ps[frame], estimates[frame, :3])

    ell_plot[0] = ax.plot_wireframe(
        ell[:, :, 0], ell[:, :, 1], ell[:, :, 2], alpha=0.2
    )
    return true_line, est_line


ani = FuncAnimation(fig, update, frames=N, interval=100, blit=False)

plt.show()
