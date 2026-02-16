import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tb3_localization_project")

    # --- Args ---
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    rviz_config = LaunchConfiguration("rviz_config")

    default_map = os.path.join(pkg_share, "maps", "ekf_map.yaml")

    # Nav2 default params file (contains AMCL params for Humble)
    default_params = os.path.join(
        get_package_share_directory("nav2_bringup"),
        "params",
        "nav2_params.yaml",
    )

    default_rviz = os.path.join(
        get_package_share_directory("nav2_bringup"),
        "rviz",
        "nav2_default_view.rviz",
    )

    # --- 1) Gazebo + TB3 spawn (already working) ---
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "ekf_world_tb3.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # --- 2) EKF local (robot_localization) ---
    # This should publish /odometry/filtered and TF odom -> base_footprint
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "ekf_local.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # --- 3) Map server ---
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "yaml_filename": map_yaml,
        }],
    )

    # --- 4) AMCL ---
    # Uses map + scan + TF(odom->base_footprint) and publishes TF(map->odom) + /amcl_pose
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        # If your scan topic is /scan already, no remap needed.
        # remappings=[("scan", "/scan")]
    )

    # --- Lifecycle manager for map_server + amcl ---
    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["map_server", "amcl"],
        }],
    )

    # --- RViz ---
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Start EKF shortly after Gazebo, then AMCL/map after EKF is up
    delayed_ekf = TimerAction(
        period=2.0,
        actions=[ekf_launch],
    )

    delayed_localization = TimerAction(
        period=3.0,
        actions=[map_server, amcl, lifecycle, rviz],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),

        sim_launch,
        delayed_ekf,
        delayed_localization,
    ])
