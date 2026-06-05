#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# Start a rosbag2 recording in a new tmux window inside the mapping session.
# Run start_rko_lio.sh first, then call this script when ready to record.
#
# Usage:
#   bash ~/tb3_3d_ws/src/turtlebot_rkolio_hardware/scripts/record_bag.sh
#
# Optional env overrides:
#   BAG_NAME_PREFIX=myrun  bash .../record_bag.sh
#
# Stop recording:
#   tmux send-keys -t mapping:record C-c
#   — or attach and press Ctrl+C in the record window
# ─────────────────────────────────────────────────────────────────────────────

SESSION=mapping
SSD=/mnt/ssd
RECORDINGS=$SSD/recordings
BAG_NAME_PREFIX=${BAG_NAME_PREFIX:-map}
BAG_TOPICS=${BAG_TOPICS:-"/velodyne_points /mavros/imu/data /imu/synced /rko_lio/odometry /rko_lio/local_map /rko_lio/frame /tf /tf_static"}

# ── Check mapping session is running ─────────────────────────────────────────
if ! tmux has-session -t $SESSION 2>/dev/null; then
  echo "ERROR: tmux session '$SESSION' not found."
  echo "  Start RKO-LIO first:"
  echo "    bash ~/tb3_3d_ws/src/turtlebot_rkolio_hardware/scripts/start_rko_lio.sh"
  exit 1
fi

# ── Always mount SSD ──────────────────────────────────────────────────────────
if ! mountpoint -q $SSD; then
  echo "=== Mounting SSD ==="
  sudo mount /dev/nvme0n1p1 $SSD
  if ! mountpoint -q $SSD; then
    echo "ERROR: Could not mount SSD. Aborting to protect eMMC storage."
    echo "  Check that the SSD is plugged in."
    exit 1
  fi
fi

mkdir -p "$RECORDINGS"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_PATH="$RECORDINGS/${BAG_NAME_PREFIX}_${TIMESTAMP}"

WS=~/tb3_3d_ws
ROS_SETUP="source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && export ROS_DOMAIN_ID=45"

# Kill any previous record window, then open a fresh one
tmux kill-window -t $SESSION:record 2>/dev/null || true
tmux new-window -t $SESSION -n record \
  "bash -c '$ROS_SETUP && ros2 bag record -o $BAG_PATH $BAG_TOPICS; exec bash'"

echo ""
echo "┌─────────────────────────────────────────────────────┐"
echo "│  Recording started (SSD)                             │"
echo "│  Output: $BAG_PATH"
echo "│                                                      │"
echo "│  Stop:   tmux send-keys -t $SESSION:record C-c       │"
echo "│  Attach: tmux select-window -t $SESSION:record       │"
echo "└─────────────────────────────────────────────────────┘"
