import numpy as np
from robot3r import PlanarRRR
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle

np.random.seed(42)

robot = PlanarRRR()
theta = np.array([0.1, 0.1, 0.1])

# model multiple points racing toward the grasp center
# if the object is moving. put the grasp back abit

rcx = 0.0
rcy = 0.0
rr = 1.0

objx = 0.0
objy = 0.0
objr = 0.0

rechx = 0.0
rechy = 0.0
rechr = 0.3


def hyperellipsoid_axis_length(cMax, cMin):  # L
    dof = 2
    r1 = cMax
    ri = cMin
    diagTerm = [r1] + [ri] * (dof - 1)
    return np.diag(diagTerm)


def unit_ball_sampling_bulk(dof=2, num_samples=1):
    u = np.random.normal(0.0, 1.0, (dof + 2, num_samples))
    norms = np.linalg.norm(u, axis=0)
    u = u / norms
    return u[:dof, :]  # The first N coordinates are uniform in a unit N ball


def unit_ball_surface_sampling_bulk(dof=2, num_samples=1):
    u = np.random.normal(0.0, 1.0, (dof, num_samples))
    norms = np.linalg.norm(u, axis=0)
    u = u / norms
    return u  # The first N coordinates are uniform on the surface of a unit N ball


def sampling_rotation_matrix(n):
    A = np.random.randn(n, n)
    Q, R = np.linalg.qr(A)
    # fix sign ambiguity
    D = np.diag(np.sign(np.diag(R)))
    Q = Q @ D
    # enforce det = +1
    if np.linalg.det(Q) < 0:
        Q[:, 0] *= -1
    return Q


RRRR = sampling_rotation_matrix(2)

xstart = np.array([0.0, 0.0])
xgoal = np.array([1.0, 1.0])
direction = 1  # +1 toward goal, -1 away

fig, ax = plt.subplots()
ax.set_xlim(-1.0, 1.0)
ax.set_ylim(-1.0, 1.0)
ax.set_aspect("equal")

(point,) = ax.plot([], [], "ro")
(line,) = ax.plot([], [], "b--")


def update_(frame):
    global xstart, direction

    delta = xgoal - xstart
    d = np.linalg.norm(delta)

    if d < 0.01:
        direction *= -1

    step = 0.01 * delta / (d + 1e-8)
    xstart += direction * step

    point.set_data([xstart[0]], [xstart[1]])  # FIX
    line.set_data([xstart[0], xgoal[0]], [xstart[1], xgoal[1]])

    return point, line


ani = animation.FuncAnimation(fig, update_, interval=50, blit=True, repeat=True)

plt.show()

raise


def mouse_move(event):
    x, y = event.xdata, event.ydata
    if x is not None and y is not None:
        objx, objy = x, y
        line.set_data([rcx, objx], [rcy, objy])
        cobj.set_center((objx, objy))
        cobj.set_radius(np.hypot(objx - rcx, objy - rcy))

        xvals, yvals = get_grasp_ort_val(objx, objy)
        grasp_or.set_data(xvals, yvals)

        # obj reach
        xcenter = np.array([objx, objy])
        Xmove = X + xcenter[:, None]
        samp.set_data(Xmove[0, :], Xmove[1, :])

        am.set_data([Xmove[0, 10], objx], [Xmove[1, 10], objy])

        # robot
        Xd = np.array([objx, objy, np.deg2rad(-75)])
        res = robot.inverse_kinematic_geometry(Xd)
        if res is not None:
            tik = res[0]  # Choose the first solution
            link_pos = robot.forward_kinematic_link(tik)
            line_link.set_data(link_pos[:, 0], link_pos[:, 1])

        crech.set_center((objx, objy))
        plt.draw()


fig, ax = plt.subplots()

(line,) = ax.plot([rcx, objx], [rcy, objy], "ro--", label="grasp center")


# grasp orientation
def get_grasp_ort_val(xpc=0, ypc=0):
    a = np.deg2rad(-75)  # angle in radians
    dx = (0.1) * np.cos(a)
    dy = (0.1) * np.sin(a)
    x_vals = [xpc - dx, xpc + dx]
    y_vals = [ypc - dy, ypc + dy]
    return x_vals, y_vals  # rotating line


xvals, yvals = get_grasp_ort_val(rcx, rcy)
(grasp_or,) = ax.plot(xvals, yvals, label="grasp orientation")

# approach grasp
xcenter = np.array([0.0, 0.0])
Rot = np.eye(2)
L = hyperellipsoid_axis_length(rechr, rechr)
X = Rot @ L @ unit_ball_surface_sampling_bulk(2, 50) + xcenter[:, None]
(samp,) = ax.plot(X[0, :], X[1, :], "bx", alpha=0.5, label="object reachability")

# approaching move
(am,) = ax.plot(
    [X[0, 10], objx], [X[1, 10], objy], "m--", alpha=0.5, label="approaching move"
)

# robot
link_pos = robot.forward_kinematic_link(theta)
(line_link,) = ax.plot(link_pos[:, 0], link_pos[:, 1], "o-", label="real robot")

qghost = np.array([2.3, -1.4, -0.9])
link_pos_ghost = robot.forward_kinematic_link(qghost)
(line_link_ghost,) = ax.plot(
    link_pos_ghost[:, 0],
    link_pos_ghost[:, 1],
    "o--",
    alpha=0.5,
    label="ghost robot",
)

c = Circle((rcx, rcy), rr, fill=False)
ax.add_patch(c)
cobj = Circle((objx, objy), objr, color="blue", fill=False)
ax.add_patch(cobj)
crech = Circle((rechx, rechy), rechr, color="green", fill=False)
ax.add_patch(crech)

# design
ax.set_xlim(-1.0, 3.0)
ax.set_ylim(-2.0, 2.0)
ax.set_aspect("equal")
ax.legend(
    loc="upper right",
)
plt.connect("motion_notify_event", mouse_move)
plt.show()
