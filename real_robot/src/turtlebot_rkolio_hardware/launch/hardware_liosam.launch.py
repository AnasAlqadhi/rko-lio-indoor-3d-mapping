"""Full LIO-SAM stack on real hardware.

Launch order:
  1. Velodyne VLP-16 + MAVROS + static TFs  (hardware_sensors.launch.py)
  2. All four LIO-SAM nodes (params from liosam_hardware.yaml)
  3. RViz (liosam_hardware.rviz)
  4. After 10 s: request 200 Hz IMU from Pixhawk (handled in sensors launch)

NOTE: No vlp16_ring_time_fixer needed — the real Velodyne driver publishes
      PointCloud2 with ring + time fields natively.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hw_pkg = get_package_share_directory('turtlebot_rkolio_hardware')

    velodyne_ip_arg = DeclareLaunchArgument(
        'velodyne_ip', default_value='192.168.4.201',
        description='Velodyne VLP-16 IP address used by the sensor bringup.')

    # ── Sensors (Velodyne + MAVROS + TFs + 200 Hz IMU request) ─────────────
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hw_pkg, 'launch', 'hardware_sensors.launch.py')
        ),
        launch_arguments={
            'velodyne_ip': LaunchConfiguration('velodyne_ip'),
        }.items(),
    )

    # ── LIO-SAM nodes ───────────────────────────────────────────────────────
    params_file = os.path.join(hw_pkg, 'config', 'liosam_hardware.yaml')
    common_params = [params_file]

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

    # ── RViz ────────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(hw_pkg, 'config', 'liosam_hardware.rviz')],
    )

    # Delay SLAM and RViz by 8 s to let sensors come up first
    delayed = TimerAction(period=8.0, actions=lio_nodes + [rviz])

    # Static TF: map → odom (identity) so the global map frame resolves in RViz
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    return LaunchDescription([
        velodyne_ip_arg,
        sensors,
        static_tf_map_odom,
        delayed,
    ])
