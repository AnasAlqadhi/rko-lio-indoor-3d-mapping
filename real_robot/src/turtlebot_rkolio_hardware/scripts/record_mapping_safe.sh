#!/bin/bash

echo "Checking if RKO-LIO is running..."

# Check if required topics exist
if ! ros2 topic list | grep -q "/rko_lio/odometry"; then
    echo "ERROR: /rko_lio/odometry not found!"
    echo "Start RKO-LIO first: ~/tb3_3d_ws/scripts/start_px4_rko_lio.sh"
    exit 1
fi

if ! ros2 topic list | grep -q "/rko_lio/local_map"; then
    echo "ERROR: /rko_lio/local_map not found!"
    echo "Start RKO-LIO first: ~/tb3_3d_ws/scripts/start_px4_rko_lio.sh"
    exit 1
fi

echo "? RKO-LIO topics detected. Starting recording..."

# Create recordings directory
RECORDINGS_DIR="$HOME/tb3_3d_ws/recordings"
mkdir -p "$RECORDINGS_DIR"

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="mapping_session_$TIMESTAMP"

echo "Recording to: $RECORDINGS_DIR/$BAG_NAME"

ros2 bag record \
    -o "$RECORDINGS_DIR/$BAG_NAME" \
    /rko_lio/odometry \
    /rko_lio/local_map \
    /velodyne_points \
    /mavros/imu/data \
    /tf \
    /tf_static

