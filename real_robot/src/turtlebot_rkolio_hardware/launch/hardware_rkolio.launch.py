"""Full RKO-LIO stack on real hardware.

Launch order:
  1. Velodyne VLP-16 + MAVROS + static TFs  (hardware_sensors.launch.py)
  2. RKO-LIO online odometry (config from rkolio_params.yaml)
  3. RViz (mapping_view.rviz)
  4. Optional rosbag2 recorder with configurable topics
    5. After 10 s: request 200 Hz IMU from Pixhawk (handled in sensors launch)
"""

import os
import re
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


DEFAULT_BAG_TOPICS = " ".join([
    "/velodyne_points",
    "/mavros/imu/data",
    "/imu/synced",
    "/rko_lio/odometry",
    "/rko_lio/local_map",
    "/rko_lio/frame",
    "/tf",
    "/tf_static",
])


def _is_true(value):
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _parse_topics(topic_string):
    return [topic for topic in re.split(r"[\s,]+", topic_string.strip()) if topic]


def _make_bag_record_action(context):
    if not _is_true(perform_substitutions(context, [LaunchConfiguration('record_bag')])):
        return []

    bag_output_dir = os.path.expanduser(
        perform_substitutions(context, [LaunchConfiguration('bag_output_dir')])
    )
    bag_name_prefix = perform_substitutions(
        context, [LaunchConfiguration('bag_name_prefix')]
    )
    bag_topics = _parse_topics(
        perform_substitutions(context, [LaunchConfiguration('bag_topics')])
    )
    bag_start_delay = float(
        perform_substitutions(context, [LaunchConfiguration('bag_start_delay')])
    )
    include_hidden_topics = _is_true(
        perform_substitutions(context, [LaunchConfiguration('bag_include_hidden_topics')])
    )

    if not bag_topics:
        raise RuntimeError('record_bag:=true requires at least one topic in bag_topics.')

    bag_path = os.path.join(
        bag_output_dir,
        f"{bag_name_prefix}_{datetime.now():%Y%m%d_%H%M%S}",
    )

    bag_cmd = ['ros2', 'bag', 'record', '-o', bag_path]
    if include_hidden_topics:
        bag_cmd.append('--include-hidden-topics')
    bag_cmd.extend(bag_topics)

    return [
        TimerAction(
            period=bag_start_delay,
            actions=[
                ExecuteProcess(
                    cmd=bag_cmd,
                    output='screen',
                    emulate_tty=True,
                )
            ],
        )
    ]


def generate_launch_description():
    hw_pkg  = get_package_share_directory('turtlebot_rkolio_hardware')

    record_bag_arg = DeclareLaunchArgument(
        'record_bag', default_value='false',
        description='Start rosbag2 recording from this launch.')
    bag_output_dir_arg = DeclareLaunchArgument(
        'bag_output_dir', default_value='/mnt/ssd/recordings',
        description='Directory where rosbag2 output folders will be created.')
    bag_name_prefix_arg = DeclareLaunchArgument(
        'bag_name_prefix', default_value='map',
        description='Prefix used for the rosbag2 output folder name.')
    bag_start_delay_arg = DeclareLaunchArgument(
        'bag_start_delay', default_value='25.0',
        description='Seconds to wait before starting rosbag2 recording.')
    bag_include_hidden_topics_arg = DeclareLaunchArgument(
        'bag_include_hidden_topics', default_value='false',
        description='Whether to pass --include-hidden-topics to rosbag2.')
    bag_topics_arg = DeclareLaunchArgument(
        'bag_topics', default_value=DEFAULT_BAG_TOPICS,
        description='Whitespace or comma separated list of topics to record.')
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

    # ── RKO-LIO odometry ────────────────────────────────────────────────────
    config_file = os.path.join(hw_pkg, 'config', 'rkolio_params.yaml')
    rkolio = Node(
        package='rko_lio',
        executable='online_node',
        name='rko_lio_node',
        output='screen',
        emulate_tty=True,
        parameters=[config_file],
    )

    # ── RViz ────────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(hw_pkg, 'config', 'mapping_view.rviz')],
    )

    # Delay SLAM by 12 s — gives IMU time to stabilize before RKO-LIO reads it
    delayed = TimerAction(period=12.0, actions=[rkolio])

    return LaunchDescription([
        record_bag_arg,
        bag_output_dir_arg,
        bag_name_prefix_arg,
        bag_start_delay_arg,
        bag_include_hidden_topics_arg,
        bag_topics_arg,
        velodyne_ip_arg,
        sensors,
        delayed,
        rviz,
        OpaqueFunction(function=_make_bag_record_action),
    ])
