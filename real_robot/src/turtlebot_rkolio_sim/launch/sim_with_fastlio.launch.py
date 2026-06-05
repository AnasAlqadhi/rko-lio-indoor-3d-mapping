"""Launch full simulation: Gazebo + custom TurtleBot + FAST-LIO + RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_dir = get_package_share_directory('turtlebot_rkolio_sim')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    tb3_gazebo_models = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'models')

    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        tb3_gazebo_models + ':' + os.path.expanduser('~/.gazebo/models') + ':' + existing_model_path,
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='corridor',
        description=(
            'World name (no .world extension). Built-in: corridor, basement_room, tunnel, '
            'static_arena, turtlebot3_world. TB3: house, dqn_stage1..4. '
            'AWS: warehouse, warehouse_no_roof, bookstore, small_house. '
            'Classic: willowgarage, cafe.'
        ),
    )

    world_file = PathJoinSubstitution([pkg_dir, 'worlds', LaunchConfiguration('world')])

    urdf_file = os.path.join(pkg_dir, 'urdf', 'custom_turtlebot.urdf.xacro')
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

    sdf_file = os.path.join(pkg_dir, 'models', 'custom_turtlebot_rkolio', 'model.sdf')
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-entity', 'custom_turtlebot_rkolio',
            '-file', sdf_file,
            '-x', '0.0', '-y', '0.0', '-z', '0.01',
        ],
    )

    # FAST-LIO world frame is camera_init. Make it the parent of odom so the
    # robot TF tree (odom→base_footprint→velodyne) and the FAST-LIO map
    # (camera_init frame) are all reachable from a single fixed frame in RViz.
    # Do NOT also publish map→odom here — that would give odom two parents.
    static_tf_camera_init_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_init_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'odom'],
    )

    fixer_node = Node(
        package='turtlebot_rkolio_sim',
        executable='vlp16_ring_time_fixer.py',
        name='vlp16_ring_time_fixer',
        output='screen',
    )

    fastlio_config = os.path.join(pkg_dir, 'config', 'fastlio_vlp16_sim.yaml')
    fastlio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[fastlio_config, {'use_sim_time': True}],
    )

    rviz_config = os.path.join(pkg_dir, 'config', 'fastlio_rviz.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    delayed_fixer_and_fastlio = TimerAction(period=5.0, actions=[fixer_node, fastlio_node])
    delayed_rviz = TimerAction(period=3.0, actions=[rviz])

    return LaunchDescription([
        gazebo_model_path,
        world_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        static_tf_camera_init_odom,
        delayed_fixer_and_fastlio,
        delayed_rviz,
    ])
