#!/bin/bash
# Sensor status monitor: shows if LiDAR and IMU are publishing
source /opt/ros/humble/setup.bash
source ~/tb3_3d_ws/install/setup.bash
export ROS_DOMAIN_ID=45

echo '========================================='
echo '       SENSOR STATUS MONITOR'
echo '  Topics: /velodyne_points | /mavros/imu/data'
echo '  Press Ctrl+C to stop'
echo '========================================='
echo ''

while true; do
    # Check publisher counts (fast, no waiting for messages)
    LIDAR_PUBS=$(ros2 topic info /velodyne_points 2>/dev/null | awk '/Publisher count/{print $3}')
    IMU_PUBS=$(ros2 topic info /mavros/imu/data 2>/dev/null | awk '/Publisher count/{print $3}')

    # Determine status
    LIDAR_STATUS='[OFF]'
    IMU_STATUS='[OFF]'
    [ "${LIDAR_PUBS:-0}" -gt 0 ] 2>/dev/null && LIDAR_STATUS='[ ON]'
    [ "${IMU_PUBS:-0}"   -gt 0 ] 2>/dev/null && IMU_STATUS='[ ON]'

    # Overall message
    if [ "$LIDAR_STATUS" = '[ ON]' ] && [ "$IMU_STATUS" = '[ ON]' ]; then
        MSG='BOTH SENSORS OK -- Ready for RKO-LIO'
    elif [ "$LIDAR_STATUS" = '[ ON]' ]; then
        MSG='Only LiDAR working  (check Pixhawk/MAVROS)'
    elif [ "$IMU_STATUS" = '[ ON]' ]; then
        MSG='Only IMU working    (check Velodyne cable/power)'
    else
        MSG='NO sensors active   (run start_sensors.sh first)'
    fi

    printf '\r[%s]  LiDAR: %s  IMU: %s  -->  %s        '         "$(date +'%H:%M:%S')" "$LIDAR_STATUS" "$IMU_STATUS" "$MSG"

    sleep 3
done
