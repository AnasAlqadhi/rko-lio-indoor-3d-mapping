#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/tb3_3d_ws/install/setup.bash

CONFIG_FILE="$HOME/tb3_3d_ws/config/px4_config.yaml"

if [ "$1" == "--rviz" ] || [ "$1" == "-r" ]; then
    echo "Starting RKO-LIO with PX4 Cube Orange+ and RViz..."
    ros2 launch rko_lio odometry.launch.py \
        config_file:="$CONFIG_FILE" \
        rviz:=true
else
    echo "Starting RKO-LIO with PX4 Cube Orange+ (use --rviz for visualization)..."
    ros2 launch rko_lio odometry.launch.py \
        config_file:="$CONFIG_FILE" \
        rviz:=false
fi
