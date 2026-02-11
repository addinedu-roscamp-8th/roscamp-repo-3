from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def default_model_path() -> str:
    return str(Path.home() / "workspace" / "fire_detetion" / "results" / "weights" / "best.pt")


def generate_launch_description():
    model_path = LaunchConfiguration("model_path")
    camera_device = LaunchConfiguration("camera_device")
    topic_name = LaunchConfiguration("topic_name")
    visualize = LaunchConfiguration("visualize")

    homography_yaml = PathJoinSubstitution(
        [FindPackageShare("top_view_fire"), "config", "homography_params.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value=default_model_path(),
            ),
            DeclareLaunchArgument("camera_device", default_value="/dev/video1"),
            DeclareLaunchArgument("topic_name", default_value="/top_view/fire_base_point"),
            DeclareLaunchArgument("visualize", default_value="false"),
            Node(
                package="top_view_fire",
                executable="fire_map_publisher",
                name="fire_map_publisher",
                output="screen",
                parameters=[
                    {
                        "model_path": model_path,
                        "camera_device": camera_device,
                        "topic_name": topic_name,
                        "visualize": visualize,
                        "homography_yaml": homography_yaml,
                    }
                ],
            ),
        ]
    )
