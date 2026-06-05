#!/bin/bash

# Configuration
DESKTOP_DIR="$HOME/Desktop"
SESSION_NAME="${1:-mapping_session_$(date +%Y%m%d_%H%M%S)}"

echo "Recording RKO-LIO mapping to Desktop..."
echo "Session: $SESSION_NAME"
echo "Location: $DESKTOP_DIR/$SESSION_NAME"
echo "Press Ctrl+C to stop recording"
echo "========================================="

# Record to Desktop
ros2 bag record -o "$DESKTOP_DIR/$SESSION_NAME" \
    --compression-mode file \
    --compression-format zstd \
    /rko_lio/odometry \
    /velodyne_points \
    /mavros/imu/data \
    /tf \
    /tf_static

echo "Recording saved to: $DESKTOP_DIR/$SESSION_NAME"
