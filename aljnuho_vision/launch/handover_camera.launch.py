from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

config = get_package_share_directory("aljnuho_vision") + "/config/"
camera_config = "camera_centroid.yaml"
with open(config + camera_config, "r") as f:
    params = yaml.safe_load(f)

params_updated = {
    "image_publisher": {
        "ros__parameters": {
            "classid": params["classid"],
            "weight": config + params["weight"],
            "cam_robot_pose": config + params["cam_robot_pose"],
            "cam_robot_matrix": config + params["cam_robot_matrix"],
            "cam_left_id": params["cam_left_id"],
            "cam_right_id": params["cam_right_id"],
            "cam_left_stereo_config": config + params["cam_left_stereo_config"],
            "cam_right_stereo_config": config + params["cam_right_stereo_config"],
        }
    }
}


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "estimate_centroid",
            default_value="true",
            description="Estimate centroid",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "estimate_allpoints",
            default_value="false",
            description="Estimate all points",
        )
    )

    ld = []
    ld.append(
        Node(
            package="aljnuho_vision",
            executable="camera_centroid_estimation",
            name="centroid_estimation_node",
            output="screen",
            parameters=[params_updated],
            condition=IfCondition(LaunchConfiguration("estimate_centroid")),
        )
    )
    ld.append(
        Node(
            package="aljnuho_vision",
            executable="camera_allpoint_estimation",
            name="allpoints_estimation_node",
            output="screen",
            parameters=[params_updated],
            condition=IfCondition(LaunchConfiguration("estimate_allpoints")),
        )
    )
    return LaunchDescription(declared_arguments + ld)
