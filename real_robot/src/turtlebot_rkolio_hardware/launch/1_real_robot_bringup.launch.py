import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ── PATHS ─────────────────────────────────────────────────────────────
    velodyne_dir = get_package_share_directory('velodyne')
    mavros_dir = get_package_share_directory('mavros')

    # ── ARGUMENTS ──────────────────────────────────────────────────────────
    # Velodyne Arguments
    device_ip_arg = DeclareLaunchArgument('device_ip', default_value='192.168.4.201')
    port_arg = DeclareLaunchArgument('port', default_value='2368')
    rpm_arg = DeclareLaunchArgument('rpm', default_value='600.0')
    model_arg = DeclareLaunchArgument('model', default_value='VLP16')

    # MAVROS Arguments
    fcu_url_arg = DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:921600')
    gcs_url_arg = DeclareLaunchArgument('gcs_url', default_value='')

    # TF Arguments (Defaults based on TB3 Waffle Pi typical stack)
    lidar_z_arg = DeclareLaunchArgument('lidar_z', default_value='0.3')
    imu_z_arg = DeclareLaunchArgument('imu_z', default_value='0.1')

    # ── VELODYNE DRIVER ───────────────────────────────────────────────────
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(velodyne_dir, 'launch', 'velodyne-all-nodes-VLP16-launch.py')
        ),
        launch_arguments={
            'device_ip': LaunchConfiguration('device_ip'),
            'port': LaunchConfiguration('port'),
            'rpm': LaunchConfiguration('rpm'),
            'model': LaunchConfiguration('model'),
        }.items()
    )

    # ── MAVROS (PX4) ──────────────────────────────────────────────────────
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mavros_dir, 'launch', 'px4.launch')
        ),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
        }.items()
    )

    # ── STATIC TRANSFORMS ─────────────────────────────────────────────────
    # base_link -> velodyne
    tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_velodyne',
        arguments=[
            '0', '0', LaunchConfiguration('lidar_z'), 
            '0', '0', '0', '1', 
            'base_link', 'velodyne'
        ]
    )

    # base_link -> imu_link
    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=[
            '0', '0', LaunchConfiguration('imu_z'), 
            '0', '0', '0', '1', 
            'base_link', 'imu_link'
        ]
    )

    return LaunchDescription([
        # Args
        device_ip_arg,
        port_arg,
        rpm_arg,
        model_arg,
        fcu_url_arg,
        gcs_url_arg,
        lidar_z_arg,
        imu_z_arg,

        # Nodes/Launch files
        velodyne_launch,
        mavros_launch,
        tf_base_to_lidar,
        tf_base_to_imu,
    ])
