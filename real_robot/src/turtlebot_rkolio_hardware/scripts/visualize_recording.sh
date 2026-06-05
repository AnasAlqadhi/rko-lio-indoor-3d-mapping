#!/bin/bash

BAG_PATH="$1"

if [ -z "$BAG_PATH" ]; then
    echo "Usage: $0 <path_to_bag_directory>"
    echo "Example: $0 ~/Desktop/mapping_session_20250917_142530"
    exit 1
fi

if [ ! -d "$BAG_PATH" ]; then
    echo "Error: Bag directory not found: $BAG_PATH"
    exit 1
fi

echo "Visualizing recording: $BAG_PATH"
echo "Starting playback and RViz..."

# Start bag playback
ros2 bag play "$BAG_PATH" --loop &
BAG_PID=$!

# Wait a moment for topics to start
sleep 2

# Start RViz
rviz2 &
RVIZ_PID=$!

echo "Playback and RViz started"
echo "In RViz:"
echo "  1. Set Fixed Frame to 'odom'"
echo "  2. Add Odometry display for /rko_lio/odometry"
echo "  3. Add PointCloud2 display for /velodyne_points (if available)"
echo ""
echo "Press Enter to stop playback and close RViz"
read

# Clean up
kill $BAG_PID $RVIZ_PID 2>/dev/null
