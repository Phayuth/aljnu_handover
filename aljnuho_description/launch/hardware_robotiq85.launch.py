from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("aljnuho_description"),
        "config",
        "robotiq85",
        "robotiq85.yaml",
    )

    gripper_node = Node(
        package="robotiq2f",
        executable="robotiq85_service",
        name="robotiq_85_node",
        output="screen",
        parameters=[config],
    )

    return LaunchDescription([gripper_node])
