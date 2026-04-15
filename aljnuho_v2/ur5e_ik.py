import numpy as np
from eaik.IK_DH import DhRobot
from pytransform3d.plot_utils import make_3d_axis
from pytransform3d.transform_manager import TransformManager
from pytransform3d.transformations import plot_transform
import matplotlib.pyplot as plt
from system_test import RobotController


np.set_printoptions(precision=4, suppress=True, linewidth=200)


class RobotUR5eKin:

    def __init__(self):
        self.d = np.array([0.1625, 0, 0, 0.1333, 0.0997, 0.0996])
        self.alpha = np.array([np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0])
        self.a = np.array([0, -0.425, -0.3922, 0, 0, 0])

        self.bot = DhRobot(self.alpha, self.a, self.d)

    def solve_fk(self, q):
        return self.bot.fwdKin(q)

    def solve_aik(self, H):
        sols = self.bot.IK(H)
        numsols = sols.num_solutions()
        Q = sols.Q
        return numsols, Q

    @staticmethod
    def _dh_transform(a, alpha, d, theta):
        """Standard DH homogeneous transform from frame i-1 to frame i."""
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        return np.array(
            [
                [ct, -st * ca, st * sa, a * ct],
                [st, ct * ca, -ct * sa, a * st],
                [0.0, sa, ca, d],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

    def get_dh_chain(self, q):
        """Return per-link DH transforms A_i and cumulative base transforms T_0i."""
        q = np.asarray(q, dtype=float)
        if q.shape != (6,):
            raise ValueError("q must be a 6-element joint vector for UR5e")

        relative_tf = []
        world_tf = [np.eye(4)]
        T = np.eye(4)

        for i in range(6):
            A_i = self._dh_transform(self.a[i], self.alpha[i], self.d[i], q[i])
            relative_tf.append(A_i)
            T = T @ A_i
            world_tf.append(T.copy())

        return relative_tf, world_tf

    def plot_link_transforms(self, q, frame_size=0.08):
        """Plot all frame transforms derived from DH parameters for a joint state."""
        relative_tf, world_tf = self.get_dh_chain(q)

        tm = TransformManager()
        for i, A_i in enumerate(relative_tf, start=1):
            tm.add_transform(f"link_{i}", f"link_{i-1}", A_i)

        ax = make_3d_axis(ax_s=1.0)

        # Plot frame axes for base and each link frame.
        for i, T_0i in enumerate(world_tf):
            plot_transform(ax=ax, A2B=T_0i, s=frame_size, name=f"L{i}")

        # Draw link-centerline segments between frame origins.
        origins = np.array([T_0i[:3, 3] for T_0i in world_tf])
        ax.plot(origins[:, 0], origins[:, 1], origins[:, 2], "k-o", linewidth=2)

        reach = np.sum(np.abs(self.a)) + np.sum(np.abs(self.d))
        lim = max(0.8, 1.1 * reach)
        ax.set_xlim([-lim, lim])
        ax.set_ylim([-lim, lim])
        ax.set_zlim([0.0, lim])
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_zlabel("Z [m]")
        ax.set_title("UR5e DH Link Transformations")
        ax.set_box_aspect([1, 1, 1])
        plt.tight_layout()
        plt.show()

        # A_chain, T_chain = robot_kin.plot_link_transforms(q)
        # for i, A_i in enumerate(A_chain, start=1):
        #     print(f"A_{i} (link_{i-1} -> link_{i}):\n", A_i)
        # for i, T_0i in enumerate(T_chain):
        #     print(f"T_0{i} (base -> link_{i}):\n", T_0i)

        return relative_tf, world_tf

    def setup_live_plot(self, frame_size=0.08):
        """Initialize live plotting environment. Call once before sending live q updates."""
        self.ax = make_3d_axis(ax_s=1.0)
        self.fig = self.ax.figure
        plt.ion()  # Enable interactive mode

        reach = np.sum(np.abs(self.a)) + np.sum(np.abs(self.d))
        lim = max(0.8, 1.1 * reach)
        self.ax.set_xlim([-lim, lim])
        self.ax.set_ylim([-lim, lim])
        self.ax.set_zlim([0.0, lim])
        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.set_zlabel("Z [m]")
        self.ax.set_box_aspect([1, 1, 1])

    def plot_link_transforms_live(self, q, frame_size=0.08):
        """Update live plot with new joint angles q. Call setup_live_plot() once first."""
        if not hasattr(self, 'ax'):
            self.setup_live_plot(frame_size)

        # Clear previous plot
        self.ax.clear()

        # Recompute DH chain
        relative_tf, world_tf = self.get_dh_chain(q)

        # Plot frame axes
        for i, T_0i in enumerate(world_tf):
            plot_transform(ax=self.ax, A2B=T_0i, s=frame_size, name=f"L{i}")

        # Draw link-centerline segments
        origins = np.array([T_0i[:3, 3] for T_0i in world_tf])
        self.ax.plot(origins[:, 0], origins[:, 1], origins[:, 2], "k-o", linewidth=2)

        # Reapply axis properties
        reach = np.sum(np.abs(self.a)) + np.sum(np.abs(self.d))
        lim = max(0.8, 1.1 * reach)
        self.ax.set_xlim([-lim, lim])
        self.ax.set_ylim([-lim, lim])
        self.ax.set_zlim([0.0, lim])
        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.set_zlabel("Z [m]")
        self.ax.set_title(f"UR5e Live: q = {np.round(q, 3)}")
        self.ax.set_box_aspect([1, 1, 1])

        # Update display without blocking
        plt.pause(0.001)

        return relative_tf, world_tf

    # @staticmethod
    # def _convert_urdf_to_dh_frame(H):
    #     "from our design task in urdf frame to dh frame"
    #     Hdh_to_urdf = SE3.Rz(np.pi).A
    #     return np.linalg.inv(Hdh_to_urdf) @ H

    # @staticmethod
    # def _convert_dh_to_urdf_frame(H):
    #     "from dh frame to our design task in urdf frame"
    #     Hdh_to_urdf = SE3.Rz(np.pi).A
    #     return Hdh_to_urdf @ H


if __name__ == "__main__":
    robot_real = RobotController()
    robot_kin = RobotUR5eKin()

    robot_real_q = robot_real.get_actual_q()
    print("Real robot joint angles (radians):\n", robot_real_q)

    # robot_real.rtde_c.teachMode()

    # robot_kin.setup_live_plot(frame_size=0.08)
    # try:
    #     while True:
    #         new_q = robot_real.get_actual_q()
    #         robot_kin.plot_link_transforms_live(new_q)
    #         # In real use: sleep or wait for new sensor data
    #         import time
    #         time.sleep(0.1)
    # except KeyboardInterrupt:
    #     print("Exiting live plot loop.")
    # robot_real.rtde_c.endTeachMode()
