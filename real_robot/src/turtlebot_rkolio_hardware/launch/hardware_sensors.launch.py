"""Common sensor bringup for real robot.

Launches:
    - Velodyne VLP-16 driver (192.168.4.201:2368)
  - MAVROS / Pixhawk Cube Orange+ (/dev/ttyACM0)
  - Static TFs: base_link → velodyne, base_link → imu_link
  - After 10 s: requests HIGHRES_IMU at 200 Hz via MAVROS service
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    velodyne_driver_dir = get_package_share_directory('velodyne_driver')
    velodyne_pointcloud_dir = get_package_share_directory('velodyne_pointcloud')
    velodyne_laserscan_dir = get_package_share_directory('velodyne_laserscan')
    mavros_dir   = get_package_share_directory('mavros')

    # ── Launch arguments ────────────────────────────────────────────────────
    velodyne_ip_arg = DeclareLaunchArgument(
        'velodyne_ip', default_value='192.168.4.201',
        description='Velodyne VLP-16 IP address')
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url', default_value='/dev/ttyACM0:921600',
        description='Pixhawk serial port and baud rate')

    # ── Velodyne VLP-16 ─────────────────────────────────────────────────────
    velodyne_driver_params = {
        'device_ip': LaunchConfiguration('velodyne_ip'),
        'frame_id': 'velodyne',
        'model': 'VLP16',
        'port': 2368,
        'read_fast': False,
        'read_once': False,
        'repeat_delay': 0.0,
        'rpm': 600.0,
    }
    velodyne_driver = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver_node',
        output='screen',
        parameters=[velodyne_driver_params],
    )

    transform_params_file = os.path.join(
        velodyne_pointcloud_dir, 'config', 'VLP16-velodyne_transform_node-params.yaml'
    )
    with open(transform_params_file, 'r', encoding='utf-8') as handle:
        transform_params = yaml.safe_load(handle)['velodyne_transform_node']['ros__parameters']
    transform_params['calibration'] = os.path.join(
        velodyne_pointcloud_dir, 'params', 'VLP16db.yaml'
    )
    velodyne_transform = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        name='velodyne_transform_node',
        output='screen',
        parameters=[transform_params],
    )

    laserscan_params_file = os.path.join(
        velodyne_laserscan_dir, 'config', 'default-velodyne_laserscan_node-params.yaml'
    )
    velodyne_laserscan = Node(
        package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        name='velodyne_laserscan_node',
        output='screen',
        parameters=[laserscan_params_file],
    )

    # ── MAVROS (Pixhawk Cube Orange+) ───────────────────────────────────────
    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(mavros_dir, 'launch', 'px4.launch')
        ),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': '',
        }.items(),
    )

    # ── Static TFs ──────────────────────────────────────────────────────────
    # base_link → velodyne: LiDAR sits 0.30 m above base_link, no rotation
    tf_base_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_velodyne',
        output='screen',
        arguments=['0', '0', '0.30', '0', '0', '0', 'base_link', 'velodyne'],
    )

    # base_link → imu_link: Pixhawk sits 0.10 m above base_link, no rotation
    tf_base_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_imu',
        output='screen',
        arguments=['0', '0', '0.10', '0', '0', '0', 'base_link', 'imu_link'],
    )

    # ── Request 200 Hz HIGHRES_IMU from Pixhawk via MAVROS ─────────────────
    # MAVLink message ID 105 = HIGHRES_IMU; 5000 µs interval = 200 Hz
    # Delayed 10 s to allow MAVROS to fully connect before calling the service.
    set_imu_200hz = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    '/mavros/set_message_interval',
                    'mavros_msgs/srv/MessageInterval',
                    '{message_id: 105, message_rate: 200.0}',
                ],
                output='screen',
            )
        ],
    )

    # ── IMU republisher (re-stamps with system time to fix Pixhawk clock offset) ─
    imu_repub = Node(
        package='turtlebot_rkolio_hardware',
        executable='imu_repub.py',
        name='imu_repub',
        output='screen',
    )

    return LaunchDescription([
        velodyne_ip_arg,
        fcu_url_arg,
        velodyne_driver,
        velodyne_transform,
        velodyne_laserscan,
        mavros,
        tf_base_velodyne,
        tf_base_imu,
        set_imu_200hz,
        imu_repub,
    ])
