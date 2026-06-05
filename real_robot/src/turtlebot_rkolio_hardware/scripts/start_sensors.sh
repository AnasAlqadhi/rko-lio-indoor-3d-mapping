#!/bin/bash

set -euo pipefail

ROS_SETUP="source /opt/ros/humble/setup.bash"
if [ -f ~/tb3_3d_ws/install/setup.bash ]; then
    ROS_SETUP="$ROS_SETUP && source ~/tb3_3d_ws/install/setup.bash"
fi

VELODYNE_IP="${VELODYNE_IP:-192.168.4.201}"
HOST_IP="${HOST_IP:-192.168.4.100}"
VELODYNE_IP_CANDIDATES=("$VELODYNE_IP" "192.168.1.201" "192.168.8.201")

echo "Starting Velodyne VLP-16 only..."

# Kill any existing velodyne processes
pkill -f velodyne || true
sleep 2

# Bring up the LiDAR link on the first ethernet-style interface we can find.
ETH_INTERFACE=$(ip link show | grep -E "^[0-9]+: (eth|enp|ens|eno)" | head -1 | cut -d: -f2 | tr -d ' ')
if [ -z "$ETH_INTERFACE" ]; then
    echo "ERROR: no Ethernet interface found (expected eno1, enp*, ens*, or eth*)"
    exit 1
fi

sudo ip link set "$ETH_INTERFACE" up
sudo ip addr del 192.168.1.100/24 dev "$ETH_INTERFACE" 2>/dev/null || true
sudo ip addr replace "$HOST_IP/24" dev "$ETH_INTERFACE"

for candidate_ip in "${VELODYNE_IP_CANDIDATES[@]}"; do
    if [ -z "$candidate_ip" ]; then
        continue
    fi
    echo -n "Pinging LiDAR ($candidate_ip)... "
    if ping -c 1 -W 2 "$candidate_ip" > /dev/null 2>&1; then
        echo "OK"
        VELODYNE_IP="$candidate_ip"
        break
    fi
    echo "FAILED"
done

if ! ping -c 1 -W 2 "$VELODYNE_IP" > /dev/null 2>&1; then
    echo "FAILED"
    exit 1
fi

# Start Velodyne with proper 10Hz configuration
bash -lc "$ROS_SETUP && ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py \
    device_ip:=$VELODYNE_IP \
    rpm:=600.0 \
    port:=2368 \
    model:=VLP16" &

# Wait for sensors to initialize
sleep 5

echo "Sensors started. Checking LiDAR rate..."
timeout 5s bash -lc "$ROS_SETUP && ros2 topic hz /velodyne_points" &

wait
echo "Available topics:"
bash -lc "$ROS_SETUP && ros2 topic list"
