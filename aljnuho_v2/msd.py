import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from robot3r import PlanarRRR
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

np.random.seed(42)


class MSDRobotEE:

    def __init__(self):
        # system params
        self.m = 1.0
        self.k_pos = 10  # normal spring
        self.k_neg = -1.0  # negative stiffness
        self.dt = 0.02
        self.c = 2.0 * np.sqrt(self.k_pos * self.m)  # critical damping (linear)

        # rotational params for 4x4 pose dynamics
        self.I_rot = 1.0
        self.k_rot = 10.0
        self.c_rot = 2.0 * np.sqrt(
            self.k_rot * self.I_rot
        )  # critical damping (angular)

    def set_rotational_stiffness(self, k_rot):
        self.k_rot = float(k_rot)
        if self.k_rot < 0.0:
            raise ValueError("k_rot must be non-negative for stable damping")
        self.c_rot = 2.0 * np.sqrt(self.k_rot * self.I_rot)

    def set_rotational_inertia(self, I_rot):
        self.I_rot = float(I_rot)
        if self.I_rot <= 0.0:
            raise ValueError("I_rot must be positive")
        self.c_rot = 2.0 * np.sqrt(self.k_rot * self.I_rot)

    def step(self, pos_vec, vel_vec, obj_vec):
        # Vectorized 3D spring-damper force: F = -k(p - p_obj) - c*v
        pos_vec = np.asarray(pos_vec, dtype=float)
        vel_vec = np.asarray(vel_vec, dtype=float)
        obj_vec = np.asarray(obj_vec, dtype=float)
        a = (-self.k_pos * (pos_vec - obj_vec) - self.c * vel_vec) / self.m
        vel_vec = vel_vec + a * self.dt
        pos_vec = pos_vec + vel_vec * self.dt
        return pos_vec, vel_vec

    def step2(self, T_ee, vel3d, T_obj):
        pos_ee = T_ee[:3, 3]
        pos_obj = T_obj[:3, 3]

        pos_n, vel_n = self.step(pos_ee, vel3d, pos_obj)

        R_ee = T_ee[:3, :3]
        R_obj = T_obj[:3, :3]

        R_desired = self.interp_Rotation(R_ee, R_obj, alpha=0.1)
        T_next = np.eye(4)
        T_next[:3, 3] = pos_n
        T_next[:3, :3] = R_desired
        return T_next, vel_n  # For simplicity, keep angular velocity unchanged

    def interp_Rotation(self, R1, R2, alpha):
        if alpha <= 0.01:
            return R1
        else:
            r1 = R.from_matrix(R1)
            r2 = R.from_matrix(R2)
            slerp = Slerp([0, 1], R.from_matrix([R1, R2]))
            r_interp = slerp(alpha)
            return r_interp.as_matrix()

    @staticmethod
    def _validate_transform(T):
        T = np.asarray(T, dtype=float)
        if T.shape != (4, 4):
            raise ValueError(f"Expected 4x4 transform, got shape {T.shape}")
        return T

    @staticmethod
    def _skew(w):
        return np.array(
            [[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]],
            dtype=float,
        )

    @classmethod
    def _so3_exp(cls, w):
        theta = np.linalg.norm(w)
        if theta < 1e-10:
            return np.eye(3) + cls._skew(w)
        k = w / theta
        K = cls._skew(k)
        return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)

    @staticmethod
    def _so3_log(R):
        tr = np.trace(R)
        cos_theta = np.clip((tr - 1.0) * 0.5, -1.0, 1.0)
        theta = np.arccos(cos_theta)
        if theta < 1e-10:
            return np.zeros(3)

        # For theta near pi, use a numerically stable axis extraction.
        if np.pi - theta < 1e-6:
            axis = np.sqrt(np.maximum((np.diag(R) + 1.0) * 0.5, 0.0))
            if R[2, 1] - R[1, 2] < 0.0:
                axis[0] = -axis[0]
            if R[0, 2] - R[2, 0] < 0.0:
                axis[1] = -axis[1]
            if R[1, 0] - R[0, 1] < 0.0:
                axis[2] = -axis[2]
            axis_norm = np.linalg.norm(axis)
            if axis_norm < 1e-10:
                return np.zeros(3)
            return theta * axis / axis_norm

        vee = np.array(
            [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]],
            dtype=float,
        )
        return 0.5 * theta / np.sin(theta) * vee

    def step_transform(self, T_ee, vel_twist, T_obj):
        # Same spring-damper model on SE(3):
        # linear:  x_ddot = -k(x - x_obj) - c*x_dot
        # angular: w_dot = -k_rot*log(R_obj * R.T) - c_rot*w
        T_ee = self._validate_transform(T_ee)
        T_obj = self._validate_transform(T_obj)
        vel_twist = np.asarray(vel_twist, dtype=float)
        if vel_twist.shape != (6,):
            raise ValueError(
                f"Expected 6D twist velocity, got shape {vel_twist.shape}"
            )

        x = T_ee[:3, 3].copy()
        R = T_ee[:3, :3].copy()
        x_obj = T_obj[:3, 3]
        R_obj = T_obj[:3, :3]

        v = vel_twist[:3].copy()
        w = vel_twist[3:].copy()

        a_lin = (-self.k_pos * (x - x_obj) - self.c * v) / self.m
        R_err = R_obj @ R.T
        rot_err = self._so3_log(R_err)
        a_ang = (-self.k_rot * rot_err - self.c_rot * w) / self.I_rot

        v = v + a_lin * self.dt
        w = w + a_ang * self.dt

        x = x + v * self.dt
        R = self._so3_exp(w * self.dt) @ R

        T_next = np.eye(4)
        T_next[:3, :3] = R
        T_next[:3, 3] = x

        vel_next = np.hstack((v, w))
        return T_next, vel_next


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
