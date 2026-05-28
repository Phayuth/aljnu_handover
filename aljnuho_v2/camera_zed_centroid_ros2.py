import yaml
import rclpy
from rclpy.node import Node
import cv2
from ultralytics import YOLO
from center_traker import Center
import numpy as np
from geometry_msgs.msg import PointStamped, Point
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from camera_zed_blah import ZedCamera


class CentroidEstimation(Node):

    def __init__(self):
        super().__init__("image_publisher")
        # vision
        self.model = YOLO("yolov8x-seg.pt")
        self.centertracker = Center()
        self.zedcam = ZedCamera()
        self.classid = [40, 41]

        # pub tf
        self.camera_frame = "zed_camera_link"
        self.base_frame = "base_link"
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.read_transform()
        self.publish_camera_to_base_tf()

        # centroid pub
        self.pointpub = self.create_publisher(PointStamped, "/centroid3d", 1)
        self.pointmsg = PointStamped()
        self.pointmsg.header.frame_id = self.base_frame

        # pub loop
        self.timer = self.create_timer(
            timer_period_sec=0.001, callback=self.timer_callback
        )

    def read_transform(self):
        yaml_path = "eye_to_hand.yaml"
        with open(yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        self.HCamleftToBaselink = np.array(
            data["Result in Matrix form (row major)"]
        ).reshape(4, 4)

    def rotation_matrix_to_quaternion(self, R):
        tr = R[0, 0] + R[1, 1] + R[2, 2]
        if tr > 0.0:
            s = np.sqrt(tr + 1.0) * 2.0
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s

        q = np.array([qx, qy, qz, qw], dtype=float)
        q /= np.linalg.norm(q)
        return q

    def publish_camera_to_base_tf(self):
        t = self.HCamleftToBaselink[:3, 3]
        q = self.rotation_matrix_to_quaternion(self.HCamleftToBaselink[:3, :3])

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.base_frame
        tf_msg.child_frame_id = self.camera_frame
        tf_msg.transform.translation.x = float(t[0])
        tf_msg.transform.translation.y = float(t[1])
        tf_msg.transform.translation.z = float(t[2])
        tf_msg.transform.rotation.x = float(q[0])
        tf_msg.transform.rotation.y = float(q[1])
        tf_msg.transform.rotation.z = float(q[2])
        tf_msg.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(tf_msg)

    def transform_point_camera_to_base(self, point_camera):
        point_camera_h = np.array(
            [point_camera[0], point_camera[1], point_camera[2], 1.0],
            dtype=np.float64,
        )
        point_base_h = self.HCamleftToBaselink @ point_camera_h
        return point_base_h[:3]

    def triangulate(self, point1, point2):
        if point1.dtype != "float64":
            point1 = point1.astype(np.float64)

        if point2.dtype != "float64":
            point2 = point2.astype(np.float64)

        point3d = cv2.triangulatePoints(
            self.zedcam.infoleft["p"],
            self.zedcam.inforight["p"],
            point1.reshape(2, -1),
            point2.reshape(2, -1),
            None,
        ).flatten()
        point3d /= point3d[-1]
        return point3d


    def timer_callback(self):
        imgl, imgr = self.zedcam.read()
        if imgl is None or imgr is None:
            return

        left_bgr = self.zedcam.zed_img_to_cv2(imgl.get_data())
        right_bgr = self.zedcam.zed_img_to_cv2(imgr.get_data())
        if left_bgr is None or right_bgr is None:
            return

        mean_left, mean_right, left_mask, right_mask = (
            self.centertracker.trackedCenterShow(
                self.model, self.classid, left_bgr, right_bgr
            )
        )

        if (mean_left is not None) and (mean_right is not None):
            point3d = self.triangulate(np.array(mean_left), np.array(mean_right))
            point3dinbase = self.transform_point_camera_to_base(point3d[:3])
            self.pointmsg.header.stamp = self.get_clock().now().to_msg()
            self.pointmsg.point = Point(
                x=point3dinbase[0], y=point3dinbase[1], z=point3dinbase[2]
            )
            self.pointpub.publish(self.pointmsg)

        # visualize
        if True:
            cv2.imshow("left", left_bgr)
            cv2.imshow("right", right_bgr)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    imagePublisherNode = CentroidEstimation()
    try:
        rclpy.spin(imagePublisherNode)
    except:
        pass
    finally:
        imagePublisherNode.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
