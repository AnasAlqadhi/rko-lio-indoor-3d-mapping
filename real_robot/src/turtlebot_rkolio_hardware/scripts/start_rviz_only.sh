#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/tb3_3d_ws/install/setup.bash

echo "Starting RViz for RKO-LIO visualization..."
rviz2 -d $(ros2 pkg prefix rko_lio)/share/rko_lio/rviz/odometry.rviz
