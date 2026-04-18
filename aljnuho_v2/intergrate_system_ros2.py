import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from intergrate_system import IntegrateSystem
from aljnuho_v2.tracker_class import UKFRiskTracker
from ur5e_ik import RobotUR5eKin, RobotController
from msd import MSDRobotEE
import numpy as np


class IntegrateSystemROS2(Node):

    def __init__(self):
        super().__init__("integrate_system_ros2")
        self.get_logger().info("IntegrateSystemROS2 node has been started.")

        self.integrate_system = IntegrateSystem()

        self.obj_sub_ = self.create_subscription(
            Odometry, "tracker_estimate", self.obj_callback, 10
        )

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
        full_cov = np.zeros((6, 6))
        full_cov[:3, :3] = np.array(msg.pose.covariance).reshape(6, 6)[:3, :3]
        full_cov[3:, 3:] = np.array(msg.twist.covariance).reshape(6, 6)[3:, 3:]


if __name__ == "__main__":
    rclpy.init()
    node = IntegrateSystemROS2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
