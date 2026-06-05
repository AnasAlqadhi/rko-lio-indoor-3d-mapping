#!/bin/bash
# run_all_replays.sh — runs all 4 SLAM replays in series, fully automated.
#
# Order:
#   1. LIO-SAM  → warehouse_06
#   2. LIO-SAM  → bookstore_01
#   3. RKO-LIO  → warehouse_06
#   4. RKO-LIO  → bookstore_01
#
# Usage:
#   cd ~/simulation_experiment
#   ./run_all_replays.sh

set -e

BAGS=~/simulation_experiment/bags
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

source /opt/ros/humble/setup.bash
source ~/tb3_3d_ws/install/setup.bash

SLAM_PID=""

# ── helper: start a SLAM launch in background and wait for it to be ready ──
start_slam() {
    local launch_file="$1"
    local ready_node="$2"   # node name to wait for (confirms SLAM is alive)

    echo ""
    echo "============================================================"
    echo "[batch] Starting SLAM: $launch_file"
    echo "============================================================"

    ros2 launch "$launch_file" &
    SLAM_PID=$!

    echo "[batch] Waiting 12s for SLAM nodes to initialize..."
    sleep 12

    echo "[batch] SLAM ready (PID $SLAM_PID)"
}

# ── helper: stop SLAM gracefully ──
stop_slam() {
    if [[ -n "$SLAM_PID" ]]; then
        echo "[batch] Stopping SLAM (PID $SLAM_PID)..."
        kill -INT "$SLAM_PID" 2>/dev/null || true
        wait "$SLAM_PID" 2>/dev/null || true
        SLAM_PID=""
        sleep 3   # let ROS 2 nodes fully shut down before next launch
        echo "[batch] SLAM stopped."
    fi
}

# ── cleanup on Ctrl+C ──
trap 'echo "[batch] Interrupted — cleaning up..."; stop_slam; exit 1' INT TERM

# ════════════════════════════════════════════════════════════════
# BLOCK 1 — LIO-SAM replays
# ════════════════════════════════════════════════════════════════
echo ""
echo "[batch] ── Run 1/4: LIO-SAM warehouse_06 ──"
start_slam "$SCRIPT_DIR/configs/replay_liosam.launch.py"
"$SCRIPT_DIR/replay_liosam.sh" "$BAGS/sim_warehouse_06" liosam_output_warehouse_06
stop_slam

echo ""
echo "[batch] ── Run 2/4: LIO-SAM bookstore_01 ──"
start_slam "$SCRIPT_DIR/configs/replay_liosam.launch.py"
"$SCRIPT_DIR/replay_liosam.sh" "$BAGS/sim_bookstore_01" liosam_output_bookstore_01
stop_slam

# ════════════════════════════════════════════════════════════════
# BLOCK 2 — RKO-LIO replays
# ════════════════════════════════════════════════════════════════
start_slam "$SCRIPT_DIR/configs/replay_rkolio.launch.py"

echo ""
echo "[batch] ── Run 3/4: RKO-LIO warehouse_06 ──"
start_slam "$SCRIPT_DIR/configs/replay_rkolio.launch.py"
"$SCRIPT_DIR/replay_rkolio.sh" "$BAGS/sim_warehouse_06" rkolio_output_warehouse_06
stop_slam

echo ""
echo "[batch] ── Run 4/4: RKO-LIO bookstore_01 ──"
start_slam "$SCRIPT_DIR/configs/replay_rkolio.launch.py"
"$SCRIPT_DIR/replay_rkolio.sh" "$BAGS/sim_bookstore_01" rkolio_output_bookstore_01
stop_slam

# ════════════════════════════════════════════════════════════════
# DONE
# ════════════════════════════════════════════════════════════════
echo ""
echo "============================================================"
echo "[batch] ALL 4 REPLAYS COMPLETE."
echo "Output bags:"
ls -lhd "$BAGS"/liosam_output_warehouse_06 "$BAGS"/liosam_output_bookstore_01 \
         "$BAGS"/rkolio_output_warehouse_06 "$BAGS"/rkolio_output_bookstore_01 2>/dev/null
echo "============================================================"
