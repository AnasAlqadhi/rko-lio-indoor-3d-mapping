"""Gazebo + robot only (no SLAM), for recording sensor bags.

Launches Gazebo with a chosen world and spawns the custom TurtleBot (Jetson + Pixhawk
+ VLP-16). Uses the custom URDF so TF tree (base_link <-> velodyne / imu_link) is
correct for downstream SLAM replay.

Usage:
    ros2 launch ~/simulation_experiment/configs/record_sim_only.launch.py world:=small_house

Then record:
    ros2 bag record /velodyne_points /mavros/imu/data /odom /tf /tf_static /clock -o <bag_name>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    sim_pkg = get_package_share_directory('turtlebot_rkolio_sim')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    tb3_gazebo_models = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'models')

    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        tb3_gazebo_models + ':' + os.path.expanduser('~/.gazebo/models') + ':' + existing_model_path,
    )

    # Bookstore retail models use file://models/... URIs (not model://), so they
    # resolve against GAZEBO_RESOURCE_PATH rather than GAZEBO_MODEL_PATH.
    # Adding ~/.gazebo here lets gzclient find the meshes.
    existing_resource_path = os.environ.get('GAZEBO_RESOURCE_PATH', '')
    gazebo_resource_path = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH',
        os.path.expanduser('~/.gazebo') + ((':' + existing_resource_path) if existing_resource_path else ''),
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='small_house',
        description='World name (no .world extension)',
    )

    world_file = PathJoinSubstitution([sim_pkg, 'worlds', LaunchConfiguration('world')])

    urdf_file = os.path.join(sim_pkg, 'urdf', 'custom_turtlebot.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': [world_file, '.world'],
            'verbose': 'true',
            'pause': 'false',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    sdf_file = os.path.join(sim_pkg, 'models', 'custom_turtlebot_rkolio', 'model.sdf')
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-entity', 'custom_turtlebot_rkolio',
            '-file', sdf_file,
            '-x', '0.0', '-y', '0.0', '-z', '0.05',
        ],
    )

    return LaunchDescription([
        gazebo_model_path,
        gazebo_resource_path,
        world_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])
