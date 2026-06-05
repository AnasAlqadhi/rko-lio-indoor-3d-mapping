#!/bin/bash
# replay_fastlio.sh — replay source bag through FAST-LIO and record output.
# Recording stops automatically when replay finishes.
#
# IMPORTANT: plays WITHOUT the bag's own /clock topic. The sim bags contain a
# recorded /clock; replaying it together with the --clock flag creates two
# competing clock sources, which makes FAST-LIO see backward time jumps
# ("lidar loop back, clear buffer") and fail every scan ("No Effective Points").
# Excluding /clock here lets --clock be the single clock source.
#
# Assumes a FAST-LIO stack is already running (fixer + fastlio_mapping), e.g.
# started via configs/replay_fastlio.launch.py.
#
# Usage:
#   ./replay_fastlio.sh <source_bag_dir> <output_bag_name>

set -e

SOURCE_BAG="${1:?Usage: $0 <source_bag_dir> <output_bag_name>}"
OUTPUT_BAG="${2:?Usage: $0 <source_bag_dir> <output_bag_name>}"

source /opt/ros/humble/setup.bash
source ~/tb3_3d_ws/install/setup.bash

OUTPUT_DIR="$(dirname "$SOURCE_BAG")/$OUTPUT_BAG"

echo "[replay_fastlio] Source : $SOURCE_BAG"
echo "[replay_fastlio] Output : $OUTPUT_DIR"
echo "[replay_fastlio] Starting recorder..."

# Record only /Odometry (sufficient for evo APE/RPE/traj). /cloud_registered
# is large and disk is limited; skip it.
ros2 bag record \
  /Odometry \
  -o "$OUTPUT_DIR" &
RECORD_PID=$!

sleep 2

echo "[replay_fastlio] Starting bag replay at 0.5x (bag /clock EXCLUDED)..."
ros2 bag play "$SOURCE_BAG" \
  --clock \
  --rate 0.5 \
  --topics /velodyne_points /mavros/imu/data

echo "[replay_fastlio] Replay finished. Stopping recorder (PID $RECORD_PID)..."
kill -INT "$RECORD_PID"
wait "$RECORD_PID" 2>/dev/null || true

echo "[replay_fastlio] Done. Output saved to: $OUTPUT_DIR"
ros2 bag info "$OUTPUT_DIR" | grep -E "Duration|Count|Topic"
