import numpy as np
import matplotlib.pyplot as plt
import rtde_control
import rtde_receive
import rtde_io
import robotiq_gripper
from eaik.IK_DH import DhRobot
import cv2
from pytransform3d.plot_utils import make_3d_axis
from pytransform3d.transform_manager import TransformManager
from pytransform3d.transformations import plot_transform

np.set_printoptions(precision=4, suppress=True, linewidth=200)


class RobotController:

    def __init__(self):
        self.hostip = "192.168.0.39"
        self.toolioport = 54321
        self.rtde_frequency = 500.0
        self.rtde_c = rtde_control.RTDEControlInterface(self.hostip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.hostip)
        self.rtde_i = rtde_io.RTDEIOInterface(self.hostip)

        self.gripper = robotiq_gripper.RobotiqGripper()
        self.gripper.connect(self.hostip, 63352)
        self.activate_gripper()

    def activate_gripper(self):
        if self.gripper.is_active():
            print("Gripper is active")
        else:
            self.gripper.activate()

    def get_actual_tcp_pose(self):
        tcp_pose = np.array(self.rtde_r.getActualTCPPose())
        H = np.eye(4)
        H[:3, :3] = cv2.Rodrigues(tcp_pose[3:6])[0]
        H[:3, 3] = tcp_pose[0:3]
        return H

    def get_actual_tcp_speed(self):
        tcp_speed = self.rtde_r.getActualTCPSpeed()
        return tcp_speed

    def make_pose_from_H(self, H):
        pose = np.zeros(6)
        pose[0:3] = H[:3, 3]
        pose[3:6] = cv2.Rodrigues(H[:3, :3])[0].flatten()
        return pose

    def get_actual_q(self):
        return self.rtde_r.getActualQ()

    def get_jacobian(self):
        return self.rtde_c.getJacobian()

    def get_joint_torques(self):
        return self.rtde_c.getJointTorques()

    def move_joints(self, q, vel=None, acc=None):
        self.rtde_c.moveJ(q)

    def move_gripper(self, position, speed=255, force=255):
        ack = self.gripper.move(position, speed, force)
        return ack


class RobotUR5eKin:

    def __init__(self):
        self.d = np.array([0.1625, 0, 0, 0.1333, 0.0997, 0.0996])
        self.alpha = np.array([np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0])
        self.a = np.array([0, -0.425, -0.3922, 0, 0, 0])
        self.bot = DhRobot(self.alpha, self.a, self.d)
        self.reach = 0.85

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

    def plot_parallel_gripper(
        self,
        ax,
        Hgrasp,
        finger_gap=0.08,
        finger_length=0.12,
        jaw_depth=0.02,
    ):
        """Plot a simple 2-finger parallel gripper.
        Finger direction: +Z axis of gripper frame.
        Opening/closing direction: X axis of gripper frame.
        """

        def transform_points(Hgrasp, points_local):
            """Transform Nx3 points from gripper frame to world frame."""
            points_h = np.hstack(
                [points_local, np.ones((points_local.shape[0], 1))]
            )
            return (Hgrasp @ points_h.T).T[:, :3]

        half_gap = 0.5 * finger_gap

        # Finger center lines in local frame.
        left_finger_local = np.array(
            [[-half_gap, 0.0, 0.0], [-half_gap, 0.0, finger_length]]
        )
        right_finger_local = np.array(
            [[half_gap, 0.0, 0.0], [half_gap, 0.0, finger_length]]
        )

        # Palm bar at z = 0 linking both fingers.
        palm_local = np.array([[-half_gap, 0.0, 0.0], [half_gap, 0.0, 0.0]])

        # Small jaw-depth lines make each finger easier to see in 3D.
        left_depth_local = np.array(
            [[-half_gap, -jaw_depth, 0.0], [-half_gap, jaw_depth, 0.0]]
        )
        right_depth_local = np.array(
            [[half_gap, -jaw_depth, 0.0], [half_gap, jaw_depth, 0.0]]
        )

        left_finger = transform_points(Hgrasp, left_finger_local)
        right_finger = transform_points(Hgrasp, right_finger_local)
        palm = transform_points(Hgrasp, palm_local)
        left_depth = transform_points(Hgrasp, left_depth_local)
        right_depth = transform_points(Hgrasp, right_depth_local)

        ax.plot(
            left_finger[:, 0],
            left_finger[:, 1],
            left_finger[:, 2],
            "r-",
            linewidth=3,
        )
        ax.plot(
            right_finger[:, 0],
            right_finger[:, 1],
            right_finger[:, 2],
            "r-",
            linewidth=3,
        )
        ax.plot(
            palm[:, 0],
            palm[:, 1],
            palm[:, 2],
            "k-",
            linewidth=2,
        )
        ax.plot(
            left_depth[:, 0],
            left_depth[:, 1],
            left_depth[:, 2],
            "k-",
            linewidth=2,
        )
        ax.plot(
            right_depth[:, 0],
            right_depth[:, 1],
            right_depth[:, 2],
            "k-",
            linewidth=2,
        )

    def plot_link_transforms(self, ax, q, frame_size=0.08):
        """Plot all frame transforms derived from DH parameters for a joint state."""
        relative_tf, world_tf = self.get_dh_chain(q)

        tm = TransformManager()
        for i, A_i in enumerate(relative_tf, start=1):
            tm.add_transform(f"link_{i}", f"link_{i-1}", A_i)

        # Plot frame axes for base and each link frame.
        for i, T_0i in enumerate(world_tf):
            plot_transform(ax=ax, A2B=T_0i, s=frame_size, name=f"L{i}")

        # Draw link-centerline segments between frame origins.
        origins = np.array([T_0i[:3, 3] for T_0i in world_tf])
        ax.plot(origins[:, 0], origins[:, 1], origins[:, 2], "k-o", linewidth=2)

        # Draw plane at 0,0,0 for reference
        plane_size = 0.85
        ax.plot([-plane_size, plane_size], [0, 0], [0, 0], "r--")
        ax.plot([0, 0], [-plane_size, plane_size], [0, 0], "g--")

        tcir = np.linspace(0, 2 * np.pi, 100)
        xcir = self.reach * np.cos(tcir)
        ycir = self.reach * np.sin(tcir)
        zcir = np.zeros_like(tcir)
        ax.plot(xcir, ycir, zcir, "b--", label="Reachable Workspace")
        ax.set_zlim([0.0, 1.0])
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_zlabel("Z [m]")

        # A_chain, T_chain = robot_kin.plot_link_transforms(q)
        # for i, A_i in enumerate(A_chain, start=1):
        #     print(f"A_{i} (link_{i-1} -> link_{i}):\n", A_i)
        # for i, T_0i in enumerate(T_chain):
        #     print(f"T_0{i} (base -> link_{i}):\n", T_0i)
        # return relative_tf, world_tf
        return ax


def servoing():
    robot_real = RobotController()

    H = robot_real.get_actual_tcp_pose()
    q0 = robot_real.get_actual_q()
    qhome = [3.14, -1.610, 1.61, -1.558, -1.562, 0.0]

    # Parameters
    vel = 0.5
    acc = 0.5
    dt = 1.0 / 500  # 2ms
    lookahead_time = 0.1
    gain = 300

    robot_real.move_joints(qhome)

    # Execute 500Hz control loop for 2 seconds, each cycle is 2ms
    for i in range(1000):
        t_start = robot_real.rtde_c.initPeriod()
        # robot_real.rtde_c.servoL(Hpose, vel, acc, dt, lookahead_time, gain)
        robot_real.rtde_c.servoJ(qhome, vel, acc, dt, lookahead_time, gain)
        qhome[0] += 0.001
        qhome[1] += 0.001
        robot_real.rtde_c.waitPeriod(t_start)

    robot_real.rtde_c.servoStop()
    robot_real.rtde_c.stopScript()


if __name__ == "__main__":
    # robot_real = RobotController()
    robot_kin = RobotUR5eKin()

    qhome = [3.14, -1.610, 1.61, -1.558, -1.562, 0.0]
    Hgrasp = np.array(
        [
            [0.0000, -1.0000, 0.0000, 0.4],
            [-1.0000, -0.0000, -0.0000, 0.4],
            [0.0000, -0.0000, -1.0000, 0.3],
            [0.0000, 0.0000, 0.0000, 1.0000],
        ]
    )

    res_ = robot_kin.solve_aik(Hgrasp)
    if res_[0] == 0:
        print("No IK solution found for the given grasp pose.")
    else:
        n, Q = res_
        print(f"Number of IK solutions found: {n}")
        print(Q)

        diff = Q - np.array(qhome)
        norms = np.linalg.norm(diff, axis=1)
        best_idx = np.argmin(norms)
        print(f"Best IK solution index: {best_idx}, q: {Q[best_idx]}")
        q = Q[best_idx]

    ax3d = make_3d_axis(ax_s=1.0)
    (obj3d_line,) = ax3d.plot([], [], [], "ro", label="Grasp Pose")
    robot_kin.plot_link_transforms(ax3d, qhome)
    robot_kin.plot_link_transforms(ax3d, q)
    robot_kin.plot_parallel_gripper(ax3d, Hgrasp)
    plt.show()
