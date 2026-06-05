"""SLAM-only launch for LIO-SAM, intended for `ros2 bag play --clock`.

Starts: robot_state_publisher (TF) + vlp16 ring/time fixer + LIO-SAM nodes + RViz.
NO Gazebo, NO robot spawning.

Usage:
    Terminal A:  ros2 launch ~/simulation_experiment/configs/replay_liosam.launch.py
    Terminal B:  ros2 bag play <bag_dir> --clock
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    sim_pkg = get_package_share_directory('turtlebot_rkolio_sim')

    urdf_file = os.path.join(sim_pkg, 'urdf', 'custom_turtlebot.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
    )

    fixer_node = Node(
        package='turtlebot_rkolio_sim',
        executable='vlp16_ring_time_fixer.py',
        name='vlp16_ring_time_fixer',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    params_file = os.path.join(sim_pkg, 'config', 'liosam_vlp16_sim.yaml')
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

    rviz_config = os.path.join(sim_pkg, 'config', 'liosam_rviz.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    delayed_slam = TimerAction(period=2.0, actions=[fixer_node] + lio_nodes)
    delayed_rviz = TimerAction(period=1.0, actions=[rviz])

    return LaunchDescription([
        robot_state_publisher,
        static_tf_map_odom,
        delayed_slam,
        delayed_rviz,
    ])
