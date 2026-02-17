from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    default_params = str(
        Path(get_package_share_directory("lunar_trafficability"))
        / "config"
        / "trafficability.params.yaml"
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to ROS 2 parameter YAML",
    )

    node = Node(
        package="lunar_trafficability",
        executable="trafficability_node",
        name="trafficability_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_file_arg, node])
