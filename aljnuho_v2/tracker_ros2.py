import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import yaml
from tracker_class import UKFRiskTracker
import numpy as np


class TrackerROS2(Node):

    def __init__(self):
        super().__init__("tracker_ros2")
        self.get_logger().info("TrackerROS2 node has been started.")

        self.dt = 0.01
        self.tracker = UKFRiskTracker()
        self.z_meas = np.zeros(6, dtype=float)
        self.prev_pos = None
        self.pub_ = self.create_publisher(Odometry, "tracker_estimate", 10)
        self.obj_ = Odometry()
        self.obj_.header.frame_id = "base_link"
        self.obj_.child_frame_id = "object"

        self.timer_ = self.create_timer(self.dt, self.timer_callback)

    def update_odom_msg(self, pos, vel, full_cov):
        self.obj_.header.stamp = self.get_clock().now().to_msg()

        self.obj_.pose.pose.position.x = pos[0]
        self.obj_.pose.pose.position.y = pos[1]
        self.obj_.pose.pose.position.z = pos[2]

        self.obj_.twist.twist.linear.x = vel[0]
        self.obj_.twist.twist.linear.y = vel[1]
        self.obj_.twist.twist.linear.z = vel[2]

        Ppos = full_cov[:3, :3]
        Pvel = full_cov[3:, 3:]
        PPos = np.zeros((6, 6))
        PVel = np.zeros((6, 6))
        PPos[:3, :3] = Ppos
        PVel[:3, :3] = Pvel

        self.obj_.pose.covariance = PPos.flatten().tolist()
        self.obj_.twist.covariance = PVel.flatten().tolist()

    def timer_callback(self):
        # from camera, already have noise
        pos = np.array([1.0, 1.0, 0.0]) + 0.1 * np.random.multivariate_normal(
            np.zeros(3), np.eye(3) * 0.01
        )
        if self.prev_pos is None:
            vel = np.zeros(3)
        else:
            vel = (pos - self.prev_pos) / self.dt
        self.prev_pos = pos.copy()
        self.z_meas[:] = np.hstack((pos, vel))

        self.tracker.ukf.predict()
        self.tracker.ukf.update(self.z_meas)

        pos = self.tracker.ukf.x[:3]
        vel = self.tracker.ukf.x[3:]
        self.update_odom_msg(pos, vel, self.tracker.ukf.P)
        self.pub_.publish(self.obj_)


if __name__ == "__main__":
    rclpy.init()
    tracker_node = TrackerROS2()
    rclpy.spin(tracker_node)
    tracker_node.destroy_node()
    rclpy.shutdown()
