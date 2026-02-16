import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tb3_localization_project")
    default_ekf_yaml = os.path.join(pkg_share, "config", "ekf_local.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    ekf_yaml = LaunchConfiguration("ekf_yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock"
        ),
        DeclareLaunchArgument(
            "ekf_yaml",
            default_value=default_ekf_yaml,
            description="Full path to EKF yaml config"
        ),

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_yaml, {"use_sim_time": use_sim_time}],
        ),
    ])
