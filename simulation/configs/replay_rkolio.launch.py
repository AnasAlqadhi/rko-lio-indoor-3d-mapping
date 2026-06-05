"""SLAM-only launch for RKO-LIO, intended for `ros2 bag play --clock`.

Starts: robot_state_publisher (TF) + RKO-LIO online node + RViz.
NO Gazebo, NO robot spawning.

Usage:
    Terminal A:  ros2 launch ~/simulation_experiment/configs/replay_rkolio.launch.py
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

    rkolio_config = os.path.join(sim_pkg, 'config', 'sim_rkolio_config.yaml')
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

    rviz_config = os.path.join(sim_pkg, 'config', 'sim_rviz.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    delayed_slam = TimerAction(period=2.0, actions=[rkolio_node])
    delayed_rviz = TimerAction(period=1.0, actions=[rviz])

    return LaunchDescription([
        robot_state_publisher,
        static_tf_map_odom,
        delayed_slam,
        delayed_rviz,
    ])
