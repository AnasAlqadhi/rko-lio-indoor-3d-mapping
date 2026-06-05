#!/bin/bash

# Create recordings directory
RECORDINGS_DIR="$HOME/tb3_3d_ws/recordings"
mkdir -p "$RECORDINGS_DIR"

# Generate filename with timestamp
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="mapping_session_$TIMESTAMP"

echo "Starting mapping recording session..."
echo "Filename: $BAG_NAME"
echo "Path: $RECORDINGS_DIR/$BAG_NAME"

# Record important topics
ros2 bag record \
    -o "$RECORDINGS_DIR/$BAG_NAME" \
    /rko_lio/odometry \
    /rko_lio/local_map \
    /velodyne_points \
    /mavros/imu/data \
    /tf \
    /tf_static

echo "Recording finished. File saved at:"
echo "$RECORDINGS_DIR/$BAG_NAME"
