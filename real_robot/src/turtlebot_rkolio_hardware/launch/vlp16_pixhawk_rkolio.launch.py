"""
vlp16_pixhawk_rkolio.launch.py
==============================
Unified ROS 2 launch file — brings up:
  1. Velodyne VLP-16 driver           (PointCloud2  @ ~10 Hz)
  2. MAVROS / Pixhawk Orange Cube+    (IMU          @ ≥200 Hz)
  3. Static TF tree  (REP-103/105)
  4. 200 Hz IMU rate request          (MAVROS service call, delayed 10 s)
  5. RKO-LIO odometry node            (delayed 12 s to allow IMU stabilisation)
  6. RViz (optional, default enabled)

Frame conventions — REP-103 / REP-105:
  map ← odom ← base_link ← imu_link
                           └────────── velodyne

Hardware geometry (adjust via launch args if your stack differs):
  base_link  — robot chassis centre
  imu_link   — Pixhawk at the same Z level as base_link  (imu_z = 0.00 m)
  velodyne   — VLP-16 mounted 15 cm above the Pixhawk    (lidar_z = 0.15 m)

Launch arguments
----------------
    velodyne_ip     IP address of the VLP-16             default: 192.168.4.201
  velodyne_port   UDP port of the VLP-16                default: 2368
  velodyne_rpm    Rotation speed (rpm)                  default: 600.0
  fcu_url         Pixhawk serial URL for MAVROS          default: /dev/ttyACM0:921600
  gcs_url         Ground control station URL (empty=off) default: ""
  lidar_z         Z-offset of velodyne above base_link   default: 0.15 (m)
  imu_z           Z-offset of imu_link above base_link   default: 0.00 (m)
  config_file     Full path to RKO-LIO params YAML       default: (package config)
  use_rviz        Launch RViz for visualisation          default: true
  rviz_config     Full path to RViz config file          default: (package config)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hw_pkg  = get_package_share_directory('turtlebot_rkolio_hardware')
    rko_pkg = get_package_share_directory('rko_lio')

    # ── Default paths ────────────────────────────────────────────────────────
    default_config  = os.path.join(hw_pkg, 'config', 'vlp16_pixhawk_icm42688p.yaml')
    default_rviz    = os.path.join(hw_pkg, 'config', 'mapping_view.rviz')

    # ── Launch arguments ─────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            'velodyne_ip', default_value='192.168.4.201',
            description='IP address assigned to the Velodyne VLP-16'),
        DeclareLaunchArgument(
            'velodyne_port', default_value='2368',
            description='UDP port the VLP-16 sends packets to'),
        DeclareLaunchArgument(
            'velodyne_rpm', default_value='600.0',
            description='VLP-16 rotation speed in rpm (600 = 10 Hz scan rate)'),
        DeclareLaunchArgument(
            'fcu_url', default_value='/dev/ttyACM0:921600',
            description='MAVROS FCU URL: <serial_port>:<baud> or udp://<ip>:<port>'),
        DeclareLaunchArgument(
            'gcs_url', default_value='',
            description='MAVROS GCS bridge URL (leave empty to disable)'),
        DeclareLaunchArgument(
            'lidar_z', default_value='0.15',
            description='Z translation of the VLP-16 above base_link (metres)'),
        DeclareLaunchArgument(
            'imu_z', default_value='0.00',
            description='Z translation of imu_link (Pixhawk) above base_link (metres)'),
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='Absolute path to the RKO-LIO params YAML file'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Set to false to skip RViz (saves CPU on embedded boards)'),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz,
            description='Absolute path to the RViz config file'),
    ]

    # ── Velodyne VLP-16 driver ───────────────────────────────────────────────
    #  The upstream 'velodyne' meta-package provides a combined launch file that
    #  starts the driver, pointcloud converter and laserscan nodes together.
    velodyne_launch_file = PathJoinSubstitution([
        FindPackageShare('velodyne'), 'launch',
        'velodyne-all-nodes-VLP16-launch.py',
    ])
    velodyne = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(velodyne_launch_file),
        launch_arguments={
            'device_ip': LaunchConfiguration('velodyne_ip'),
            'port':      LaunchConfiguration('velodyne_port'),
            'rpm':       LaunchConfiguration('velodyne_rpm'),
            'model':     'VLP16',
            'frame_id':  'velodyne',      # REP-103: sensor-specific name
        }.items(),
    )

    # ── MAVROS (Pixhawk Orange Cube Plus — PX4 or ArduPilot) ─────────────────
    #  px4.launch works for both PX4 and ArduPilot when fcu_url is correct.
    #  For ArduPilot swap to: os.path.join(mavros_dir, 'launch', 'apm.launch')
    mavros_launch_file = PathJoinSubstitution([
        FindPackageShare('mavros'), 'launch', 'px4.launch',
    ])
    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(mavros_launch_file),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
        }.items(),
    )

    # ── Static TFs (REP-103 / REP-105) ───────────────────────────────────────
    #
    #  publish:  base_link → velodyne
    #            base_link → imu_link
    #
    #  static_transform_publisher args:
    #    x y z  yaw pitch roll  parent_frame  child_frame
    #  (RPY = 0 = identity rotation — parallel mounting assumed)

    tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_velodyne',
        output='screen',
        # The VLP-16 is offset +lidar_z metres along Z; no rotation.
        arguments=[
            '0', '0', LaunchConfiguration('lidar_z'),
            '0', '0', '0', '1',          # qx qy qz qw
            'base_link', 'velodyne',
        ],
    )

    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        output='screen',
        # The Pixhawk is at +imu_z metres along Z; REP-103 alignment assumed.
        arguments=[
            '0', '0', LaunchConfiguration('imu_z'),
            '0', '0', '0', '1',          # qx qy qz qw
            'base_link', 'imu_link',
        ],
    )

    # ── Request 200 Hz HIGHRES_IMU from the Pixhawk ──────────────────────────
    #  MAVLink message ID 105 = HIGHRES_IMU
    #  Interval = 5000 µs → 200 Hz
    #  Delayed 10 s so that MAVROS has time to establish the MAVLink heartbeat
    #  before the service is called.
    #
    #  Prerequisite on Pixhawk side (ArduPilot):
    #    IMU_GYRO_RATEMAX = 200 (or 400 for higher-rate boards)
    #    SR*_EXTRA1 = 0  (let MAVROS set the rate via SET_MESSAGE_INTERVAL)
    #
    #  Prerequisite on Pixhawk side (PX4):
    #    IMU_INTEG_RATE = 200 (EKF2 integration rate)
    #    MAV_0_RATE = 0       (unlimited; let MAVROS negotiate)
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
                name='set_highres_imu_200hz',
            )
        ],
    )

    # ── RKO-LIO odometry node ────────────────────────────────────────────────
    #  Delayed 12 s:
    #   • 0–10 s  : MAVROS + Velodyne initialise
    #   • 10 s    : 200 Hz IMU rate is requested
    #   • 10–12 s : Pixhawk EKF2 receives the new rate and stabilises
    #   • 12 s+   : RKO-LIO begins processing
    rkolio_node = Node(
        package='rko_lio',
        executable='online_node',
        name='rko_lio_node',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            # Explicit remappings — the params file already sets these topics,
            # but remappings act as a belt-and-suspenders guarantee.
            ('/imu',            '/mavros/imu/data'),
            ('/lidar',          '/velodyne_points'),
        ],
    )

    rkolio_delayed = TimerAction(period=12.0, actions=[rkolio_node])

    # ── RViz (optional) ──────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    # ── Assemble LaunchDescription ────────────────────────────────────────────
    return LaunchDescription(
        args + [
            # Sensors
            velodyne,
            mavros,
            # TF tree
            tf_base_to_lidar,
            tf_base_to_imu,
            # Rate request
            set_imu_200hz,
            # SLAM — delayed
            rkolio_delayed,
            # Visualisation
            rviz,
        ]
    )
