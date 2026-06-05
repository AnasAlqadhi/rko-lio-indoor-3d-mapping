#!/bin/bash
# replay_rkolio.sh — replay source bag through RKO-LIO and record output.
# Recording stops automatically when replay finishes.
#
# Usage:
#   ./replay_rkolio.sh <source_bag_dir> <output_bag_name>
#
# Example:
#   ./replay_rkolio.sh ~/simulation_experiment/bags/sim_small_house_06 rkolio_output_small_house_06

set -e

SOURCE_BAG="${1:?Usage: $0 <source_bag_dir> <output_bag_name>}"
OUTPUT_BAG="${2:?Usage: $0 <source_bag_dir> <output_bag_name>}"

source /opt/ros/humble/setup.bash
source ~/tb3_3d_ws/install/setup.bash

OUTPUT_DIR="$(dirname "$SOURCE_BAG")/$OUTPUT_BAG"

echo "[replay_rkolio] Source : $SOURCE_BAG"
echo "[replay_rkolio] Output : $OUTPUT_DIR"
echo "[replay_rkolio] Starting recorder..."

ros2 bag record \
  /rko_lio/odometry \
  /rko_lio/local_map \
  -o "$OUTPUT_DIR" &
RECORD_PID=$!

# Give the recorder a moment to open the database
sleep 2

echo "[replay_rkolio] Starting bag replay at 0.5x speed..."
ros2 bag play "$SOURCE_BAG" \
  --clock \
  --rate 0.5 \
  --topics /velodyne_points /mavros/imu/data /clock

echo "[replay_rkolio] Replay finished. Stopping recorder (PID $RECORD_PID)..."
kill -INT "$RECORD_PID"
wait "$RECORD_PID" 2>/dev/null || true

echo "[replay_rkolio] Done. Output saved to: $OUTPUT_DIR"
ros2 bag info "$OUTPUT_DIR" | grep -E "Duration|Count|Topic"
