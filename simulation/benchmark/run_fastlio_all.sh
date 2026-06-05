#!/bin/bash
# run_fastlio_all.sh — run FAST-LIO replay on all 3 world bags, fully automated.
# Fresh FAST-LIO stack per world (so the map never carries over).
#
# Usage:  cd ~/simulation_experiment && ./run_fastlio_all.sh

set -e
BAGS=~/simulation_experiment/bags
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/humble/setup.bash
source ~/tb3_3d_ws/install/setup.bash

SLAM_PID=""

FIXER_PID=""
start_slam() {
    echo "[fastlio-batch] launching headless FAST-LIO stack (no RViz)..."
    # robot_state_publisher for TF (custom URDF)
    ros2 run robot_state_publisher robot_state_publisher \
        --ros-args -p use_sim_time:=true \
        -p robot_description:="$(xacro ~/tb3_3d_ws/src/turtlebot_rkolio_sim/urdf/custom_turtlebot.urdf.xacro)" \
        > ~/simulation_experiment/logs/fastlio_batch_rsp.log 2>&1 &
    ros2 run turtlebot_rkolio_sim vlp16_ring_time_fixer.py --ros-args -p use_sim_time:=true \
        > ~/simulation_experiment/logs/fastlio_batch_fixer.log 2>&1 &
    FIXER_PID=$!
    ros2 run fast_lio fastlio_mapping \
        --ros-args --params-file ~/tb3_3d_ws/src/turtlebot_rkolio_sim/config/fastlio_vlp16_sim.yaml \
        -p use_sim_time:=true \
        > ~/simulation_experiment/logs/fastlio_batch_slam.log 2>&1 &
    SLAM_PID=$!
    sleep 10
    echo "[fastlio-batch] FAST-LIO ready (fastlio PID $SLAM_PID)"
}

stop_slam() {
    if [[ -n "$SLAM_PID" ]]; then
        echo "[fastlio-batch] stopping FAST-LIO (PID $SLAM_PID)..."
        kill -INT "$SLAM_PID" 2>/dev/null || true
        wait "$SLAM_PID" 2>/dev/null || true
        # belt-and-braces: kill stragglers
        for p in $(pgrep -f fastlio_mapping) $(pgrep -f vlp16_ring_time) $(pgrep -f robot_state_publisher); do kill -9 $p 2>/dev/null || true; done
        SLAM_PID=""; FIXER_PID=""
        sleep 4
        echo "[fastlio-batch] stopped."
    fi
}

trap 'echo "[fastlio-batch] interrupted"; stop_slam; exit 1' INT TERM

for W in small_house_06 warehouse_06 bookstore_01; do
    echo ""
    echo "============================================================"
    echo "[fastlio-batch] FAST-LIO → $W"
    echo "============================================================"
    rm -rf "$BAGS/fastlio_output_$W"
    start_slam
    "$SCRIPT_DIR/replay_fastlio.sh" "$BAGS/sim_$W" "fastlio_output_$W"
    stop_slam
done

echo ""
echo "============================================================"
echo "[fastlio-batch] ALL FAST-LIO REPLAYS COMPLETE."
ls -lhd "$BAGS"/fastlio_output_* 2>/dev/null
echo "============================================================"
