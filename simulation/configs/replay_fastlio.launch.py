"""SLAM-only launch for FAST-LIO, intended for `ros2 bag play --clock`.

Starts: robot_state_publisher (TF) + vlp16 ring/time fixer + FAST-LIO + RViz.
NO Gazebo, NO robot spawning.

FAST-LIO consumes /velodyne_points_proc (ring+time from the fixer) and
/mavros/imu/data, and publishes odometry on /Odometry in the camera_init frame.

Usage:
    Terminal A:  ros2 launch ~/simulation_experiment/configs/replay_fastlio.launch.py
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

    # FAST-LIO world frame is camera_init. Tie it to odom so RViz can show
    # both the robot TF tree and the FAST-LIO map under one fixed frame.
    static_tf_camera_init_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_init_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'odom'],
        parameters=[{'use_sim_time': True}],
    )

    fixer_node = Node(
        package='turtlebot_rkolio_sim',
        executable='vlp16_ring_time_fixer.py',
        name='vlp16_ring_time_fixer',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    fastlio_config = os.path.join(sim_pkg, 'config', 'fastlio_vlp16_sim.yaml')
    fastlio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[fastlio_config, {'use_sim_time': True}],
    )

    rviz_config = os.path.join(sim_pkg, 'config', 'fastlio_rviz.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    delayed_slam = TimerAction(period=2.0, actions=[fixer_node, fastlio_node])
    delayed_rviz = TimerAction(period=1.0, actions=[rviz])

    return LaunchDescription([
        robot_state_publisher,
        static_tf_camera_init_odom,
        delayed_slam,
        delayed_rviz,
    ])
