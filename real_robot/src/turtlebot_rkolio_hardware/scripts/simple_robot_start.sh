#!/bin/bash
# Simple robot startup without network complications

echo "Starting Robot System (Simplified)"
echo "=================================="

# Set basic environment
export ROS_DOMAIN_ID=45
source /opt/ros/humble/setup.bash
source ~/tb3_3d_ws/install/setup.bash

# Check Velodyne connection
if ping -c 1 192.168.4.201 > /dev/null; then
    echo "✓ Velodyne connected"
else
    echo "✗ Velodyne not reachable at 192.168.4.201"
    exit 1
fi

# Start your existing working system
echo "Starting existing RKO-LIO system..."
~/tb3_3d_ws/scripts/start_px4_rko_lio.sh

