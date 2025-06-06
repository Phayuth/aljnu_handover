import rclpy
from rclpy.node import Node
import cv2
from ultralytics import YOLO
from center_traker import Center
from camera_system import StereoProcess
import numpy as np
from geometry_msgs.msg import PointStamped, Point
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import json


class CentroidEstimation(Node):

    def __init__(self):
        super().__init__("image_publisher")
        self.declare_parameter("classid")
        self.declare_parameter("weight")
        self.declare_parameter("cam_robot_pose")
        self.declare_parameter("cam_robot_matrix")
        self.declare_parameter("cam_left_id")
        self.declare_parameter("cam_right_id")
        self.declare_parameter("cam_left_stereo_config")
        self.declare_parameter("cam_right_stereo_config")

        # vision
        self.classes = self.get_parameter("classid").value
        self.model = YOLO(self.get_parameter("weight").value)
        self.centertracker = Center()
        self.stereo = StereoProcess(
            self.get_parameter("cam_left_id").value,
            self.get_parameter("cam_right_id").value,
            self.get_parameter("cam_left_stereo_config").value,
            self.get_parameter("cam_right_stereo_config").value,
        )
        # pub tf
        self.HstereoTobase = None
        self.tb = StaticTransformBroadcaster(self)
        self.cam_to_base_tf()

        # centroid pub
        self.pointpub = self.create_publisher(PointStamped, "/centroid3d", 1)
        self.pointmsg = PointStamped()
        self.pointmsg.header.frame_id = "base"

        # pub loop
        self.timer = self.create_timer(timer_period_sec=0.001, callback=self.timer_callback)

    def cam_to_base_tf(self):
        with open(self.get_parameter("cam_robot_pose").value, "r") as f:
            self.stereoTobasePose = json.load(f)

        with open(self.get_parameter("cam_robot_matrix").value, "r") as ff:
            self.HstereoTobase = json.load(ff)
            self.HstereoTobase = np.array(self.HstereoTobase)

        statictf = TransformStamped()
        statictf.header.stamp = self.get_clock().now().to_msg()
        statictf.header.frame_id = "base"
        statictf.child_frame_id = "stereo_optical"
        statictf.transform.translation.x = self.stereoTobasePose[0]
        statictf.transform.translation.y = self.stereoTobasePose[1]
        statictf.transform.translation.z = self.stereoTobasePose[2]
        statictf.transform.rotation.x = self.stereoTobasePose[3]
        statictf.transform.rotation.y = self.stereoTobasePose[4]
        statictf.transform.rotation.z = self.stereoTobasePose[5]
        statictf.transform.rotation.w = self.stereoTobasePose[6]
        self.tb.sendTransform(statictf)

    def draw_over_image(self, rgb_image, uv):
        draw_cx, draw_cy = uv
        if draw_cx != 0 and draw_cy != 0:
            cv2.circle(rgb_image, (draw_cx, draw_cy), 5, (255, 0, 255), -1)
            cv2.putText(rgb_image, text=f"{draw_cx}, {draw_cy}", org=(draw_cx, draw_cy), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, thickness=1, color=(255, 0, 255))

    def timer_callback(self):
        _, imgrraw = self.stereo.camright.read()
        _, imglraw = self.stereo.camleft.read()

        imglund, imgrund = self.stereo.undistort_image(imglraw, imgrraw)

        mean_left, mean_right = self.centertracker.trackedCenterShow(self.model, self.classes, imglund, imgrund)

        if (mean_left is not None) and (mean_right is not None):
            point3d = self.stereo.triangulate(np.array(mean_left), np.array(mean_right))
            point3d = self.HstereoTobase @ point3d

            self.pointmsg.header.stamp = self.get_clock().now().to_msg()
            self.pointmsg.point = Point(x=point3d[0], y=point3d[1], z=point3d[2])
            self.pointpub.publish(self.pointmsg)

        # visualize
        if True:
            cv2.imshow("right", imgrund)
            cv2.imshow("left", imglund)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.stereo.camright.release()
                self.stereo.camleft.release()
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
