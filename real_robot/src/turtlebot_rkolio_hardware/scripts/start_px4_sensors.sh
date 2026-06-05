#!/bin/bash

set -euo pipefail

ROS_SETUP="source /opt/ros/humble/setup.bash"
if [ -f ~/tb3_3d_ws/install/setup.bash ]; then
    ROS_SETUP="$ROS_SETUP && source ~/tb3_3d_ws/install/setup.bash"
fi

VELODYNE_IP="${VELODYNE_IP:-192.168.4.201}"
VELODYNE_IP_CANDIDATES=("$VELODYNE_IP" "192.168.1.201" "192.168.8.201")

echo "Starting PX4 Cube Orange+ and Velodyne sensors..."

# Kill existing processes
pkill -f mavros || true
pkill -f velodyne || true
sleep 3

# Start PX4 MAVROS
echo "Starting PX4 MAVROS..."
bash -lc "$ROS_SETUP && ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:921600" &

# Wait for MAVROS to initialize
sleep 8

# Start Velodyne
echo "Starting Velodyne VLP-16..."
for candidate_ip in "${VELODYNE_IP_CANDIDATES[@]}"; do
    if [ -z "$candidate_ip" ]; then
        continue
    fi
    if ping -c 1 -W 2 "$candidate_ip" > /dev/null 2>&1; then
        VELODYNE_IP="$candidate_ip"
        break
    fi
done
bash -lc "$ROS_SETUP && ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py \
    device_ip:=$VELODYNE_IP \
    rpm:=600.0 \
    port:=2368 \
    model:=VLP16" &

sleep 5

echo "All sensors started!"
echo "Available topics:"
bash -lc "$ROS_SETUP && ros2 topic list | grep -E \"(mavros/imu|velodyne)\""

echo "Checking data rates..."
timeout 5s bash -lc "$ROS_SETUP && ros2 topic hz /mavros/imu/data" &
timeout 5s bash -lc "$ROS_SETUP && ros2 topic hz /velodyne_points" &
wait
