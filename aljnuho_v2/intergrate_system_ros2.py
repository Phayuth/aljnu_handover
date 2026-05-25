import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from intergrate_system import IntegrateSystem
from tracker_class import UKFRiskTracker
from ur5e_ik import RobotController
from msd import MSDRobotEE
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class IntegrateSystemROS2(Node):

    def __init__(self):
        super().__init__("integrate_system_ros2")
        self.get_logger().info("IntegrateSystemROS2 node has been started.")
        self.cbg = ReentrantCallbackGroup()  # req cb
        self.cbgtimer = ReentrantCallbackGroup()  # main loop cb
        self.ccb = ReentrantCallbackGroup()  # realtime data update cb

        self.robot_real = RobotController()
        # self.robot_kin = RobotUR5eKin()
        self.tracker = UKFRiskTracker()
        self.eemodel = MSDRobotEE()
        self.intsyst = IntegrateSystem()

        # live control parameters
        self.vel = 0.5
        self.acc = 0.5
        self.dt = 1.0 / 500  # 2ms
        self.lokat = 0.1
        self.gain = 300

        self.rot_step_deg = 5.0
        self.Hhome = self.robot_real.get_actual_tcp_pose()
        self.qcurrent = self.robot_real.get_actual_q()
        self.Hcurrent = self.robot_real.get_actual_tcp_pose()
        self.speed = np.zeros(3)
        self.speed6d = np.zeros(6)
        self.Rdesired = self.Hhome[0:3, 0:3]

        self.g_fullopen = 0
        self.g_fullclose = 255
        self.g_close = 190

        self.Hplace = np.array(
            [
                [0.0, -1.0, 0.0, 0.4],
                [-1.0, -0.0, -0.0, 0.4],
                [0.0, -0.0, -1.0, 0.3],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        # self.qplace = None

        self.obj_sub_ = self.create_subscription(
            Odometry,
            "tracker_estimate",
            self.obj_callback,
            10,
            callback_group=self.ccb,
        )
        self.timer_ = self.create_timer(
            0.005,
            self.run_loop,
            callback_group=self.cbgtimer,
        )
        self.reach_mode = {"follow_ellipse": False}
        self.real_action = {
            "move": False,
            "grip_delta": 0,
            "rot_x_delta": 0.0,
            "grip_pos": 0,
            "move_to_place": False,
            "obj_grasped": False,
            "move_chase_object": True,
            "move_to_home": False,
        }

        self.robot_real.move_joints(self.intsyst.qhome)
        self.robot_real.move_gripper(self.g_fullopen)

    def obj_callback(self, msg):
        pos = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ]
        )
        vel = np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ]
        )
        self.x = np.hstack((pos, vel))
        self.P = np.zeros((6, 6))
        self.P[:3, :3] = np.array(msg.pose.covariance).reshape(6, 6)[:3, :3]
        self.P[3:, 3:] = np.array(msg.twist.covariance).reshape(6, 6)[3:, 3:]

    def run_loop(self):
        if not self.P:
            return

        t_start = self.robot_real.rtde_c.initPeriod()

        # ellipse model
        cov_e_w, cov_e_h, cov_e_d, cov_e_angle_xy, cov_e_angle_xz = (
            self.tracker.pose_covariance_ellipse_3d(self.P)
        )
        speed_xyz = np.linalg.norm(self.x[3:6])
        rk_e_w, rk_e_h, rk_e_d, rk_e_angle_xy, rk_e_angle_xz = (
            self.tracker.risk_covariance_ellipse_3d(self.P, speed=speed_xyz)
        )

        # detect if object is out of workspace boundary
        if not self.intsyst.is_obj_in_boundary(
            (self.x[0], self.x[1], self.x[2]), self.intsyst.rworkspace
        ):
            radius_bound = self.intsyst.rreach
        else:
            radius_bound = self.intsyst.rworkspace

        # grasp point projection and reach projection
        xproj, yproj, zproj = self.intsyst.project_point_on_rotated_ellipse3d(
            center=(self.x[0], self.x[1], self.x[2]),
            width=rk_e_w,
            height=rk_e_h,
            depth=rk_e_d,
            angle_xy_deg=rk_e_angle_xy,
            angle_xz_deg=rk_e_angle_xz,
            target_point=(self.Hhome[0, 3], self.Hhome[1, 3], self.Hhome[2, 3]),
        )
        xreach, yreach, zreach, self.reach_mode["follow_ellipse"] = (
            self.intsyst.select_reach_target_model_3d(
                ee_point=(self.Hhome[0, 3], self.Hhome[1, 3], self.Hhome[2, 3]),
                ellipse_proj_point=(xproj, yproj, zproj),
                radius=radius_bound,
                follow_ellipse=self.reach_mode["follow_ellipse"],
            )
        )
        Hcurrent = self.robot_real.get_actual_tcp_pose()

        if self.real_action["move_chase_object"]:
            obj_pos = self.x[:3]
            reach_pos = np.array([xreach, yreach, zreach])
            Hdesired = self.intsyst.make_grasp_pose(
                obj_P=obj_pos,
                reach_P=reach_pos,
                ee_pose=Hcurrent,
            )
            Hctrl, speed = self.eemodel.step2(
                T_ee=Hcurrent,
                vel3d=speed,
                T_obj=Hdesired,
            )
            pose_cl = self.robot_real.make_pose_from_H(Hctrl)
        else:
            pose_cl = self.robot_real.make_pose_from_H(Hcurrent)  # hold pose

        self.robot_real.rtde_c.servoL(
            pose_cl, self.vel, self.acc, self.dt, self.lokat, self.gain
        )
        self.robot_real.rtde_c.waitPeriod(t_start)


if __name__ == "__main__":
    rclpy.init()
    node = IntegrateSystemROS2()
    exe = MultiThreadedExecutor()
    exe.add_node(node)
    try:
        exe.spin()
    finally:
        node.robot_real.rtde_c.servoStop()
        node.robot_real.rtde_c.stopScript()
        node.destroy_node()
        exe.shutdown()
    rclpy.shutdown()
