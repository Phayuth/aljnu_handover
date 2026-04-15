import numpy as np
from vpsto.vpsto import VPSTO, VPSTOOptions
from vpsto.vptraj import VPTraj
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from shapely.geometry import Polygon, MultiPolygon, LineString


# Define some polygons for creating a collision environment
class CollisionEnvironment:
    def __init__(self):
        self.poly_list = []
        self.poly_list.append(
            np.array(
                [[0.1, 0.13], [0.23, 0.12], [0.19, 0.28], [0.1, 0.32], [0.16, 0.2]]
            )
        )
        self.poly_list.append(
            np.array(
                [
                    [0.25, 0.34],
                    [0.31, 0.35],
                    [0.32, 0.41],
                    [0.27, 0.44],
                    [0.23, 0.4],
                ]
            )
        )
        self.poly_list.append(
            np.array(
                [
                    [0.35, 0.12],
                    [0.38, 0.1],
                    [0.41, 0.11],
                    [0.42, 0.21],
                    [0.35, 0.24],
                ]
            )
        )
        self.multi_poly = MultiPolygon(
            [
                Polygon(self.poly_list[0]),
                Polygon(self.poly_list[1]),
                Polygon(self.poly_list[2]),
            ]
        )

    def getTrajDist(self, pts):
        return self.multi_poly.intersection(LineString(pts)).length


q_min = 0.0 * np.ones(2)
q_max = 0.5 * np.ones(2)


def loss_limits(candidates):
    q = candidates["pos"]
    d_min = np.maximum(np.zeros_like(q), -q + q_min)
    d_max = np.maximum(np.zeros_like(q), q - q_max)
    return np.sum(d_min > 0.0, axis=(1, 2)) + np.sum(d_max > 0.0, axis=(1, 2))


env = CollisionEnvironment()


def loss_collision(candidates):
    costs = []
    for traj in candidates["pos"]:
        costs.append(env.getTrajDist(traj))
    costs = np.array(costs)
    costs += costs > 0.0
    return costs


def loss_curvature(candidates):
    dq = candidates["vel"]
    ddq = candidates["acc"]
    dq_sq = np.sum(dq**2, axis=-1)
    ddq_sq = np.sum(ddq**2, axis=-1)
    dq_ddq = np.sum(dq * ddq, axis=-1)
    return np.mean((dq_sq * ddq_sq - dq_ddq**2) / (dq_sq**3 + 1e-6), axis=-1)


xd = 0.48
yd = 0.45
tolerance = 1e-3
w_angle = 5.0
theta_star = 2.0


def loss_target(candidates):
    q = candidates["pos"]
    costs = np.abs(q[:, -1, 0] - xd) + np.abs(q[:, -1, 1] - yd)
    costs += costs > tolerance
    return costs


def loss_angle(candidates):
    # trajs: (P, N, 2)
    trajs = candidates["pos"]
    v = trajs[:, -1] - trajs[:, -2]  # (P,2)
    v_norm = v / (np.linalg.norm(v, axis=1, keepdims=True) + 1e-8)
    d_star = np.array([np.cos(theta_star), np.sin(theta_star)])
    dot = np.sum(v_norm * d_star, axis=1)
    L_angle = 1 - dot
    return w_angle * L_angle


def loss(candidates):
    cost_curvature = loss_curvature(candidates)
    cost_collision = loss_collision(candidates)
    cost_limits = loss_limits(candidates)
    cost_target = loss_target(candidates)
    cost_angle = loss_angle(candidates)
    return (
        candidates["T"]
        + 1e-3 * cost_curvature
        # + 1e3 * cost_collision
        + 1e3 * cost_limits
        + 1e2 * cost_target
        + 1e2 * cost_angle
    )


opt = VPSTOOptions(ndof=2)
opt.vel_lim = np.array([0.1, 0.1])
opt.acc_lim = np.array([0.5, 0.5])
opt.N_via = 5
opt.N_eval = 100
opt.pop_size = 25
opt.max_iter = 200
opt.sigma_init = 1.5

traj_opt = VPSTO(opt)
q0 = np.array([0.15, 0.2])  # robot position
sol = traj_opt.minimize(loss, q0=q0)

t_traj = np.linspace(0, sol.T_best, 1000)
pos, vel, acc = sol.get_posvelacc(t_traj)

print("Movement duration: ", sol.T_best)
# raise

fig, ax = plt.subplots()
ax.set_xlim([q_min[0], q_max[0]])
ax.set_ylim([q_min[1], q_max[1]])
ax.scatter(q0[0], q0[1])
for pol in env.poly_list:
    ax.add_patch(patches.Polygon(pol, facecolor="gray"))
ax.plot(pos[:, 0], pos[:, 1])
ax.plot(xd, yd, "x", color="red", markersize=10)
ax.plot(
    [xd, xd + 1 * np.cos(theta_star)],
    [yd, yd + 1 * np.sin(theta_star)],
    "-",
    color="blue",
    markersize=10,
)
plt.show()