import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from kdlrobot import KDLChain
import PyKDL as kdl


class AddmittanceController(Node):
    def __init__(self):
        super().__init__("addmittance_controller")
        self.get_logger().info("Addmittance Controller Node has been started.")

        self.setup_kdl_chain()

        self.joint_positions = [0.0] * self.kdl_chain.jointnum

        jac = self.kdl_chain.jacobian(self.joint_positions)
        self.get_logger().info(f"Jacobian: {jac}")

    def setup_kdl_chain(self):
        self.client = self.create_client(GetParameters, "/robot_state_publisher/get_parameters")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /robot_state_publisher/get_parameters service...")

        request = GetParameters.Request()
        request.names = ["robot_description"]

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.values:
                urdf_string = response.values[0].string_value
            else:
                self.get_logger().error("Failed to retrieve 'robot_description'.")
        else:
            self.get_logger().error(f"Service call failed: {future.exception()}")

        root = "ur5e_base_link"
        tip = "robotiq_85_tip"
        self.kdl_chain = KDLChain.from_string(urdf_string, root, tip)
        self.get_logger().info(f"KDL Chain created with root: {root} and tip: {tip}")
        self.get_logger().info(f"Number of joints: {self.kdl_chain.jointnum}")

if __name__ == "__main__":
    rclpy.init()
    node = AddmittanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
