import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # --- Packages ---
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # --- Default world (your world inside your package) ---
    # Put ekf.world here:
    # tb3_localization_project/worlds/ekf.world
    pkg_this = get_package_share_directory('tb3_localization_project')
    default_world = os.path.join(pkg_this, 'worlds', 'ekf.world')

    # --- Launch arguments ---
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    tb3_model = LaunchConfiguration('tb3_model')

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path to world file to load'
    )
    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set "false" to run Gazebo headless (no client)'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_tb3_model = DeclareLaunchArgument(
        'tb3_model',
        default_value=os.environ.get('TURTLEBOT3_MODEL', 'burger'),
        description='TurtleBot3 model: burger / waffle / waffle_pi'
    )

    declare_x_pose = DeclareLaunchArgument('x_pose', default_value='4.829719')
    declare_y_pose = DeclareLaunchArgument('y_pose', default_value='2.077093')
    declare_z_pose = DeclareLaunchArgument('z_pose', default_value='0.000445')
    declare_yaw_pose = DeclareLaunchArgument('yaw_pose', default_value='-3.129473')

    # --- Set TB3 model env var (turtlebot3_gazebo uses this) ---
    set_tb3_model_env = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value=tb3_model
    )

    # --- Gazebo server ---
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # --- Gazebo client (optional) ---
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui)
    )

    # --- Robot state publisher (TF from URDF) ---
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # --- Spawn TurtleBot3 ---
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose,
            'yaw_pose': yaw_pose
        }.items()
    )

    # Delay spawn slightly so Gazebo is fully ready
    delayed_spawn = TimerAction(period=2.0, actions=[spawn_turtlebot_cmd])

    # --- LaunchDescription ---
    ld = LaunchDescription()

    ld.add_action(declare_world)
    ld.add_action(declare_gui)
    ld.add_action(declare_use_sim_time)

    ld.add_action(declare_tb3_model)

    ld.add_action(declare_x_pose)
    ld.add_action(declare_y_pose)
    ld.add_action(declare_z_pose)
    ld.add_action(declare_yaw_pose)

    ld.add_action(set_tb3_model_env)

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(delayed_spawn)

    return ld
