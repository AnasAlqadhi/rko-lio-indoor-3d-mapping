"""Launch full simulation: Gazebo + custom TurtleBot + LIO-SAM + RViz."""

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

    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # Preprocessor: adds ring + time fields that LIO-SAM requires.
    # Gazebo ray sensor only publishes x/y/z/intensity.
    # Subscribes /velodyne_points → publishes /velodyne_points_proc.
    fixer_node = Node(
        package='turtlebot_rkolio_sim',
        executable='vlp16_ring_time_fixer.py',
        name='vlp16_ring_time_fixer',
        output='screen',
    )

    params_file = os.path.join(pkg_dir, 'config', 'liosam_vlp16_sim.yaml')
    common_params = [params_file, {'use_sim_time': True}]

    lio_nodes = [
        Node(package='lio_sam', executable='lio_sam_imuPreintegration',
             name='lio_sam_imuPreintegration', parameters=common_params, output='screen'),
        Node(package='lio_sam', executable='lio_sam_imageProjection',
             name='lio_sam_imageProjection', parameters=common_params, output='screen'),
        Node(package='lio_sam', executable='lio_sam_featureExtraction',
             name='lio_sam_featureExtraction', parameters=common_params, output='screen'),
        Node(package='lio_sam', executable='lio_sam_mapOptimization',
             name='lio_sam_mapOptimization', parameters=common_params, output='screen'),
    ]

    rviz_config = os.path.join(pkg_dir, 'config', 'liosam_rviz.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    delayed_liosam = TimerAction(period=5.0, actions=[fixer_node] + lio_nodes)
    delayed_rviz = TimerAction(period=3.0, actions=[rviz])

    return LaunchDescription([
        gazebo_model_path,
        world_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        static_tf_map_odom,
        delayed_liosam,
        delayed_rviz,
    ])
