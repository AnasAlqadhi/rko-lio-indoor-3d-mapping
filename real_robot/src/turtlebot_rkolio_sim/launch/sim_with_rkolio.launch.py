"""Launch full simulation: Gazebo + robot + RKO-LIO + RViz."""

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

    # Extend GAZEBO_MODEL_PATH so all world model:// URIs resolve
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        tb3_gazebo_models + ':' + os.path.expanduser('~/.gazebo/models') + ':' + existing_model_path,
    )

    # ── Launch arguments ─────────────────────────────────────────────────
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

    # ── Process URDF ─────────────────────────────────────────────────────
    urdf_file = os.path.join(pkg_dir, 'urdf', 'custom_turtlebot.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()

    # ── 1. Gazebo ────────────────────────────────────────────────────────
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

    # ── 2. Robot state publisher ─────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # ── 3. Spawn robot (use the SDF — has proven-working diff_drive/physics) ─
    sdf_file = os.path.join(pkg_dir, 'models', 'custom_turtlebot_rkolio', 'model.sdf')
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-entity', 'custom_turtlebot_rkolio',
            '-file', sdf_file,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
        ],
    )

    # ── 4. RKO-LIO online node ───────────────────────────────────────────
    rkolio_config = os.path.join(pkg_dir, 'config', 'sim_rkolio_config.yaml')
    rkolio_node = Node(
        package='rko_lio',
        executable='online_node',
        name='rko_lio_online',
        output='screen',
        parameters=[rkolio_config, {'use_sim_time': True}],
        remappings=[
            ('pointcloud', '/velodyne_points'),
            ('imu', '/mavros/imu/data'),
        ],
    )

    # ── 5. Static TF: odom → map (identity, for RViz) ───────────────────
    static_tf_odom_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_map',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # ── 6. RViz2 ─────────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_dir, 'config', 'sim_rviz.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    # Delay RKO-LIO and RViz to let Gazebo start first
    delayed_rkolio = TimerAction(period=5.0, actions=[rkolio_node])
    delayed_rviz = TimerAction(period=3.0, actions=[rviz])

    return LaunchDescription([
        gazebo_model_path,
        world_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        static_tf_odom_map,
        delayed_rkolio,
        delayed_rviz,
    ])
