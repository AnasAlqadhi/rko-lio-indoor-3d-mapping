#!/bin/bash
# Robot environment setup - FIXED

# ROS2 Configuration - Use default RMW
export ROS_DOMAIN_ID=45
# Remove problematic RMW setting
# export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# Robot-specific paths  
export ROBOT_WS="$HOME/tb3_3d_ws"
export ROBOT_CONFIG_DIR="$ROBOT_WS/config"
export ROBOT_SCRIPTS_DIR="$ROBOT_WS/scripts"
export ROBOT_RECORDINGS_DIR="$ROBOT_WS/recordings"

# Sensor configuration
export ROBOT_VELODYNE_IP="192.168.4.201"
export ROBOT_IMU_TOPIC="/mavros/imu/data"
export ROBOT_LIDAR_TOPIC="/velodyne_points"

# Create directories
mkdir -p "$ROBOT_CONFIG_DIR" "$ROBOT_SCRIPTS_DIR" "$ROBOT_RECORDINGS_DIR"

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source "$ROBOT_WS/install/setup.bash"

echo "Robot environment configured:"
echo "  Workspace: $ROBOT_WS"
echo "  Velodyne IP: $ROBOT_VELODYNE_IP" 
echo "  Domain ID: $ROS_DOMAIN_ID"
