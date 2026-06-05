"""Launch Gazebo with custom TurtleBot RKO-LIO robot in a chosen world."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('turtlebot_rkolio_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    # Stock waffle_pi URDF for robot_state_publisher TF tree
    urdf_file = os.path.join(tb3_gazebo_dir, 'urdf', 'turtlebot3_waffle_pi.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ── Launch arguments ─────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world', default_value='corridor',
        description='World: corridor, basement_room, tunnel',
    )
    x_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    y_arg = DeclareLaunchArgument('y_pose', default_value='0.0')

    # ── Gazebo server + client (separate, like stock TB3) ────────────────
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': [os.path.join(pkg_dir, 'worlds/'),
                       LaunchConfiguration('world'), '.world'],
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    )

    # ── Robot state publisher ────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # ── Spawn custom SDF model (proven approach, like stock TB3) ─────────
    sdf_file = os.path.join(
        pkg_dir, 'models', 'custom_turtlebot_rkolio', 'model.sdf')
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-entity', 'custom_turtlebot_rkolio',
            '-file', sdf_file,
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', '0.01',
        ],
    )

    return LaunchDescription([
        world_arg, x_arg, y_arg,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
    ])
