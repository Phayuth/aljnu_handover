import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tracker_class import UKFRiskTracker
import numpy as np
from geometry_msgs.msg import PointStamped


class TrackerROS2(Node):

    def __init__(self):
        super().__init__("tracker_ros2")
        self.get_logger().info("TrackerROS2 node has been started.")

        self.dt = 0.005
        self.tracker = UKFRiskTracker()
        self.z_meas = np.zeros(6, dtype=float)
        self.prev_pos = None

        self.sub_ = self.create_subscription(
            PointStamped, "centroid3d", self.centroid_callback, 10
        )
        self.pub_ = self.create_publisher(Odometry, "tracker_estimate", 10)
        self.obj_ = Odometry()
        self.obj_.header.frame_id = "base_link"
        self.obj_.child_frame_id = "object"

        # self.camera_frame = "camera_link"
        # self.base_frame = "base_link"
        # self.tf_broadcaster = StaticTransformBroadcaster(self)
        # self.publish_camera_to_base_tf()

        self.timer_ = self.create_timer(self.dt, self.timer_callback)

    # def read_transform(self):
    #     yaml_path = Path(__file__).resolve().parent / "eye_to_hand.yaml"
    #     with yaml_path.open("r", encoding="utf-8") as f:
    #         data = yaml.safe_load(f)
    #     HbaseToCam = np.array(data["Result in Matrix form (row major)"]).reshape(
    #         4, 4
    #     )
    #     return HbaseToCam

    # def rotation_matrix_to_quaternion(self, R):
    #     tr = R[0, 0] + R[1, 1] + R[2, 2]
    #     if tr > 0.0:
    #         s = np.sqrt(tr + 1.0) * 2.0
    #         qw = 0.25 * s
    #         qx = (R[2, 1] - R[1, 2]) / s
    #         qy = (R[0, 2] - R[2, 0]) / s
    #         qz = (R[1, 0] - R[0, 1]) / s
    #     elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
    #         s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
    #         qw = (R[2, 1] - R[1, 2]) / s
    #         qx = 0.25 * s
    #         qy = (R[0, 1] + R[1, 0]) / s
    #         qz = (R[0, 2] + R[2, 0]) / s
    #     elif R[1, 1] > R[2, 2]:
    #         s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
    #         qw = (R[0, 2] - R[2, 0]) / s
    #         qx = (R[0, 1] + R[1, 0]) / s
    #         qy = 0.25 * s
    #         qz = (R[1, 2] + R[2, 1]) / s
    #     else:
    #         s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
    #         qw = (R[1, 0] - R[0, 1]) / s
    #         qx = (R[0, 2] + R[2, 0]) / s
    #         qy = (R[1, 2] + R[2, 1]) / s
    #         qz = 0.25 * s

    #     q = np.array([qx, qy, qz, qw], dtype=float)
    #     q /= np.linalg.norm(q)
    #     return q

    # def publish_camera_to_base_tf(self):
    #     HbaseToCam = self.read_transform()
    #     HCamToBase = np.linalg.inv(HbaseToCam)
    #     t = HCamToBase[:3, 3]
    #     q = self.rotation_matrix_to_quaternion(HCamToBase[:3, :3])

    #     tf_msg = TransformStamped()
    #     tf_msg.header.stamp = self.get_clock().now().to_msg()
    #     tf_msg.header.frame_id = self.camera_frame
    #     tf_msg.child_frame_id = self.base_frame
    #     tf_msg.transform.translation.x = float(t[0])
    #     tf_msg.transform.translation.y = float(t[1])
    #     tf_msg.transform.translation.z = float(t[2])
    #     tf_msg.transform.rotation.x = float(q[0])
    #     tf_msg.transform.rotation.y = float(q[1])
    #     tf_msg.transform.rotation.z = float(q[2])
    #     tf_msg.transform.rotation.w = float(q[3])

    #     self.tf_broadcaster.sendTransform(tf_msg)
    #     self.get_logger().info(
    #         f"Published static TF {self.camera_frame} -> {self.base_frame} "
    #     )
    def centroid_callback(self, msg):
        # from camera sensor
        self.z_meas[:3] = np.array([msg.point.x, msg.point.y, msg.point.z])

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
        if self.prev_pos is None:
            vel = np.zeros(3)
        else:
            vel = (self.z_meas[:3] - self.prev_pos) / self.dt
        self.prev_pos = self.z_meas[:3].copy()
        self.z_meas[3:6] = vel

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
