import rclpy
from rclpy.node import Node
import PyKDL
import urdf_parser_py.urdf as urdf
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from robotiq2f_interfaces.srv import Robotiq2FCmd, Robotiq2FInfo


class RealTimeRobot(Node):

    def __init__(self):
        super().__init__("real_time_robot")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("robot_root", "")
        self.declare_parameter("robot_tip", "")

        self.cbg = ReentrantCallbackGroup()  # req cb
        self.cbgtimer = ReentrantCallbackGroup()  # main loop cb
        self.ccb = ReentrantCallbackGroup()  # realtime data update cb

        self.robot_description = self.get_parameter("robot_description").get_parameter_value().string_value
        self.robot_root = self.get_parameter("robot_root").get_parameter_value().string_value
        self.robot_tip = self.get_parameter("robot_tip").get_parameter_value().string_value
        self.robo = urdf.URDF.from_xml_string(self.robot_description)

        self.r85cli = self.create_client(Robotiq2FCmd, "/gripper_comamnd")

    def send_req_gripper(self, pos, vel, force):
        rqm = Robotiq2FCmd.Request()
        rqm.pos = pos
        rqm.vel = vel
        rqm.force = force
        future = self.r85cli.call_async(rqm)
        # rclpy.spin_until_future_complete(self, future)
        # return future.result()

    def send_req_gripper_info(self):
        rqm = Robotiq2FInfo.Request()
        future = self.r85cli.call_async(rqm)
        # rclpy.spin_until_future_complete(self, future)
        # return future.result()

    def close_gripper(self):
        self.send_req_gripper(0, 255, 255)
        self.get_logger().info("close gripper")

    def open_gripper(self):
        self.send_req_gripper(255, 255, 255)
        self.get_logger().info("open gripper")



def main(args=None):
    rclpy.init(args=args)
    node = RealTimeRobot()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()