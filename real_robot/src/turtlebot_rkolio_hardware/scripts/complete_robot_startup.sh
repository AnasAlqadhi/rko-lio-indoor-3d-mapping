#!/bin/bash
# Complete robot system startup

echo "========================================="
echo "Starting Complete Robot System"
echo "========================================="

# Step 1: Setup environment
echo "Step 1: Setting up robot environment..."
source ~/tb3_3d_ws/scripts/setup_robot_environment.sh

# Step 2: Setup network  
echo "Step 2: Setting up robot network..."
~/tb3_3d_ws/scripts/setup_robot_network.sh

# Step 3: Check PX4/MAVROS
echo "Step 3: Checking PX4/MAVROS..."
if ! ros2 topic list | grep -q "/mavros/imu/data"; then
    echo "Starting PX4 MAVROS..."
    ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:921600 &
    sleep 10
fi
echo "✓ PX4/MAVROS is running"

# Step 4: Start Velodyne
echo "Step 4: Starting Velodyne LiDAR..."
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py \
    device_ip:=$ROBOT_VELODYNE_IP \
    rpm:=600.0 \
    port:=2368 \
    model:=VLP16 &

VELODYNE_PID=$!
sleep 8

# Step 5: Verify sensors
echo "Step 5: Verifying sensor data..."
if ! ros2 topic hz $ROBOT_LIDAR_TOPIC --timeout 5; then
    echo "ERROR: Velodyne not publishing data"
    kill $VELODYNE_PID
    exit 1
fi
echo "✓ All sensors working"

# Step 6: Start static transforms  
echo "Step 6: Starting coordinate transforms..."
ros2 run tf2_ros static_transform_publisher \
    0 0 -0.3 0 0 0 1 \
    base_link velodyne &

# Step 7: Start RKO-LIO
echo "Step 7: Starting RKO-LIO..."
ros2 launch rko_lio odometry.launch.py \
    config_file:="$ROBOT_CONFIG_DIR/robot_ground_config.yaml" \
    rviz:=true

echo "Robot system startup complete!"
