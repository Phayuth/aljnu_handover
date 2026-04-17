import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from robot3r import PlanarRRR

np.random.seed(42)


class MSDRobotEE:

    def __init__(self):
        # system params
        self.m = 1.0
        self.k_pos = 10  # normal spring
        self.k_neg = -1.0  # negative stiffness
        self.dt = 0.02
        self.c = 2.2 * np.sqrt(self.k_pos * self.m)  # critical damping

    def step(self, pos_vec, vel_vec, obj_vec):
        # Vectorized 3D spring-damper force: F = -k(p - p_obj) - c*v
        pos_vec = np.asarray(pos_vec, dtype=float)
        vel_vec = np.asarray(vel_vec, dtype=float)
        obj_vec = np.asarray(obj_vec, dtype=float)
        a = (-self.k_pos * (pos_vec - obj_vec) - self.c * vel_vec) / self.m
        vel_vec = vel_vec + a * self.dt
        pos_vec = pos_vec + vel_vec * self.dt
        return pos_vec, vel_vec


if __name__ == "__main__":
    eesystem = MSDRobotEE()
    robot = PlanarRRR()
    qghost = np.array([2.3, -1.4, -0.9])
    link_pos_ghost = robot.forward_kinematic_link(qghost)
    eeposefixed = np.array([link_pos_ghost[-1, 0], link_pos_ghost[-1, 1], 0.0])
    eepose = eeposefixed.copy()

    # initial state
    pos = eepose.copy()
    vel = np.zeros(3)
    obj = np.zeros(3)
    r_obj_to_pos = np.linalg.norm(obj - pos)
    r_reaching = 1.0

    fig, ax = plt.subplots()
    creaching = Circle(
        (0, 0),
        r_reaching,
        color="green",
        fill=False,
        linestyle="--",
        label="reaching radius",
    )
    ax.add_patch(creaching)

    (line_link_ghost,) = ax.plot(
        link_pos_ghost[:, 0],
        link_pos_ghost[:, 1],
        "o--",
        alpha=0.5,
        label="ghost robot",
    )

    cir = Circle(
        (obj[0], obj[1]),
        r_obj_to_pos,
        color="blue",
        fill=False,
        label="ee to object",
    )
    ax.add_patch(cir)
    cir2 = Circle(
        (obj[0], obj[1]), 0.0, color="red", fill=False, label="base to object"
    )
    ax.add_patch(cir2)
    (pt_pos,) = ax.plot([], [], "o", label="robot ee")
    (obj_pos,) = ax.plot([], [], "x", color="red", label="object")
    (eefixed,) = ax.plot(
        [eeposefixed[0], pos[0]],
        [eeposefixed[1], pos[1]],
        "s--",
        color="purple",
    )
    (ee_to_obj,) = ax.plot(
        [pos[0], obj[0]],
        [pos[1], obj[1]],
        "d--",
        color="orange",
        label="ee to object",
    )

    ax.axhline(0, color="gray", linestyle="--")
    ax.axvline(0, color="gray", linestyle="--")
    ax.set_xlim(-1.0, 3.0)
    ax.set_ylim(-2.0, 2.0)
    ax.set_aspect("equal")
    ax.legend(loc="upper right", bbox_to_anchor=(1.15, 1))

    def mouse_move(event):
        global obj
        x, y = event.xdata, event.ydata
        if x is not None and y is not None:
            obj[:] = [x, y, 0.0]

    def change_k(event):
        if event.key == "up":
            eesystem.k_pos = 2.0
            ax.set_title(f"Stiffness: {eesystem.k_pos}, attract to object")
        elif event.key == "down":
            eesystem.k_pos = eesystem.k_neg
            ax.set_title(f"Stiffness: {eesystem.k_pos}, repel from object")
        elif event.key == "right":
            eesystem.k_pos = 0.0
            ax.set_title(f"Stiffness: {eesystem.k_pos}, no movement")

        fig.canvas.draw_idle()

    def loop():
        global pos, vel, obj
        r_obj_to_pos = np.linalg.norm(obj - pos)
        cir.set_center((obj[0], obj[1]))
        cir.set_radius(r_obj_to_pos)
        cir2.set_center((obj[0], obj[1]))
        cir2.set_radius(np.linalg.norm(obj[:2]))

        pos, vel = eesystem.step(pos, vel, obj)

        # attempt ik for all approach points
        # Xd = np.array([xobj, yobj, np.deg2rad(-75)])
        # res = robot.inverse_kinematic_geometry(Xd)
        # if res is not None:
        # tik = res[0]  # Choose the first solution

        # limit the position to be within the reaching radius
        pos_norm = np.linalg.norm(pos)
        if pos_norm >= r_reaching and pos_norm > 0.0:
            pos = pos / pos_norm * r_reaching
            vel[:] = 0.0

        # Keep simulation in x-y plane while state remains 3D.
        pos[2] = 0.0
        obj[2] = 0.0

        pt_pos.set_data([pos[0]], [pos[1]])
        obj_pos.set_data([obj[0]], [obj[1]])
        eefixed.set_data([eeposefixed[0], pos[0]], [eeposefixed[1], pos[1]])
        ee_to_obj.set_data([pos[0], obj[0]], [pos[1], obj[1]])
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("motion_notify_event", mouse_move)
    fig.canvas.mpl_connect("key_press_event", change_k)
    timer = fig.canvas.new_timer(interval=int(eesystem.dt * 1000))
    timer.add_callback(loop)
    timer.start()
    plt.show()
