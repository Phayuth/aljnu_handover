import rclpy
from rclpy.node import Node
from intergrate_system import IntegrateSystem
from tracker_class import UKFRiskTracker
from ur5e_ik import RobotController
from msd import MSDRobotEE
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class RobotLoop(Node):

    def __init__(self):
        super().__init__("robot_loop")
        self.get_logger().info("RobotLoop node has been started.")
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

        self.g_fullopen = 0
        self.g_fullclose = 255
        self.g_close = 190

    def loop(self):
        t_start = self.robot_real.rtde_c.initPeriod()
        if self.servo_l:
            self.robot_real.rtde_c.servoL(
                self.Hcurrent, self.vel, self.acc, self.dt, self.lokat, self.gain
            )
        elif self.servo_j:
            self.robot_real.rtde_c.servoJ(
                self.qcurrent, self.vel, self.acc, self.dt, self.lokat, self.gain
            )
        self.robot_real.rtde_c.waitPeriod(t_start)


if __name__ == "__main__":
    rclpy.init()
    robot_loop_node = RobotLoop()
    executor = MultiThreadedExecutor()
    executor.add_node(robot_loop_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        robot_loop_node.robot_real.rtde_c.servoStop()
        robot_loop_node.robot_real.rtde_c.stopScript()
    finally:
        robot_loop_node.destroy_node()
        rclpy.shutdown()
