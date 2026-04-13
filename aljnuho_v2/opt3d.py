import numpy as np
import matplotlib.pyplot as plt
from pytransform3d.plot_utils import make_3d_axis
from pytransform3d.transformations import plot_transform


# # cone 3d
# head = np.array([0, 0, 0])
# n = np.array([0, 0, 1])
# cone_length = 0.5
# cone_radius = 0.3


import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

# parameters
A = np.array([0.0, 0.0, 0.0])  # apex
v = np.array([-1.0, -1.0, -1.0])  # axis (unit)
theta = np.deg2rad(30)
cos2 = np.cos(theta) ** 2

# initial point (outside cone)
P0 = np.array([1.5, 1.5, 1.0])


# penalty (no normalization)
def cone_penalty(P):
    u = P - A
    dot = np.dot(u, v)
    g = cos2 * np.dot(u, u) - dot**2  # >0 outside
    forward = -dot  # penalize behind apex

    return max(0.0, g) + max(0.0, forward)


# objective = just penalty
def objective(P):
    return cone_penalty(P)


# optimize
res = minimize(objective, P0, method="BFGS")

P_opt = res.x

print("Initial:", P0)
print("Optimized:", P_opt)
print("Final penalty:", cone_penalty(P_opt))


ax = make_3d_axis(1)
plot_transform(ax, np.eye(4), s=0.1, name="World")
ax.quiver(*np.hstack([A,v]).tolist(), length=0.5)
ax.plot(P0[0], P0[1], P0[2], "ro", label="Initial Point")
ax.plot(P_opt[0], P_opt[1], P_opt[2], "go", label="Optimized Point")
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
ax.set_zlim(0, 3)
ax.legend()
plt.show()
