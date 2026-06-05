#!/bin/bash
# Run FAST-LIO on warehouse_06 and bookstore_01 (clean-clock, headless, fresh map each).
set -e
BAGS=~/simulation_experiment/bags
LOGS=~/simulation_experiment/logs
source /opt/ros/humble/setup.bash
source ~/tb3_3d_ws/install/setup.bash
export ROS_DOMAIN_ID=30

run_world() {
    local W="$1"
    echo "=================== FAST-LIO → $W ==================="
    rm -rf "$BAGS/fastlio_output_$W"
    ros2 run turtlebot_rkolio_sim vlp16_ring_time_fixer.py --ros-args -p use_sim_time:=true > "$LOGS/flr_${W}_fixer.log" 2>&1 &
    ros2 run fast_lio fastlio_mapping --ros-args --params-file ~/tb3_3d_ws/src/turtlebot_rkolio_sim/config/fastlio_vlp16_sim.yaml -p use_sim_time:=true > "$LOGS/flr_${W}_node.log" 2>&1 &
    sleep 10
    ros2 bag record /Odometry -o "$BAGS/fastlio_output_$W" > "$LOGS/flr_${W}_record.log" 2>&1 &
    sleep 3
    ros2 bag play "$BAGS/sim_$W" --clock --rate 0.5 --topics /velodyne_points /mavros/imu/data > "$LOGS/flr_${W}_play.log" 2>&1
    sleep 4
    pkill -INT -f "bag record" 2>/dev/null || true; sleep 3
    for p in $(pgrep -f fastlio_mapping) $(pgrep -f vlp16_ring_time) $(pgrep -f 'bag record'); do kill -9 $p 2>/dev/null || true; done
    sleep 3
    echo "[$W] done. loop-back=$(grep -c 'loop back' "$LOGS/flr_${W}_node.log") No-Eff=$(grep -c 'No Effective Points' "$LOGS/flr_${W}_node.log")"
}

run_world warehouse_06
run_world bookstore_01
echo "=================== ALL REMAINING FAST-LIO RUNS COMPLETE ==================="
