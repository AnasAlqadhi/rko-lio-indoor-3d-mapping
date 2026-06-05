#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# Jetson one-shot mapping startup
#   Window 0 (slam)   → Velodyne + MAVROS + RKO-LIO + RViz
#
# Usage: bash ~/tb3_3d_ws/src/turtlebot_rkolio_hardware/scripts/start_rko_lio.sh
#   Attach:   tmux attach -t mapping
#   Stop all: tmux kill-session -t mapping
# ─────────────────────────────────────────────────────────────────────────────

WS=~/tb3_3d_ws
SSD=/mnt/ssd
RECORDINGS=$SSD/recordings
SESSION=mapping
DEFAULT_BAG_TOPICS="/velodyne_points /mavros/imu/data /imu/synced /rko_lio/odometry /rko_lio/local_map /rko_lio/frame /tf /tf_static"

BAG_NAME_PREFIX=${BAG_NAME_PREFIX:-map}
BAG_START_DELAY=${BAG_START_DELAY:-25.0}
BAG_INCLUDE_HIDDEN_TOPICS=${BAG_INCLUDE_HIDDEN_TOPICS:-false}
BAG_TOPICS=${BAG_TOPICS:-$DEFAULT_BAG_TOPICS}

# ── Always mount SSD ──────────────────────────────────────────────────────────
# NOTE: sudo will prompt for your device password. For unattended startup,
# configure passwordless sudo for the specific mount/ip commands (see docs).
if ! mountpoint -q $SSD; then
  echo "=== Mounting SSD ==="
  sudo mount /dev/nvme0n1p1 $SSD
  if ! mountpoint -q $SSD; then
    echo "ERROR: Could not mount SSD. Recordings would go to eMMC — aborting."
    echo "  Check that the SSD is plugged in."
    exit 1
  fi
fi
mkdir -p "$RECORDINGS"

# ── Ask whether to record ─────────────────────────────────────────────────────
if [ -z "$RECORD_BAG" ]; then
  echo ""
  read -r -p "Start rosbag recording? [y/N]: " _ans
  case "$_ans" in
    [yY][eE][sS]|[yY]) RECORD_BAG=true  ;;
    *)                  RECORD_BAG=false ;;
  esac
fi

BAG_OUTPUT_DIR=${BAG_OUTPUT_DIR:-$RECORDINGS}

# ── When started over SSH, restore desktop env vars so RViz shows on screen ──
if [ -z "$DISPLAY" ] && pgrep -u "$(id -u)" -f "/usr/bin/gnome-shell" >/dev/null 2>&1; then
  export DISPLAY=:0
  export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
  export XDG_RUNTIME_DIR="/run/user/$(id -u)"
  export DBUS_SESSION_BUS_ADDRESS="unix:path=${XDG_RUNTIME_DIR}/bus"
fi

ROS_SETUP="source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && export ROS_DOMAIN_ID=45"

LAUNCH_ARGS=(
  "record_bag:=$RECORD_BAG"
  "bag_output_dir:=$BAG_OUTPUT_DIR"
  "bag_name_prefix:=$BAG_NAME_PREFIX"
  "bag_start_delay:=$BAG_START_DELAY"
  "bag_include_hidden_topics:=$BAG_INCLUDE_HIDDEN_TOPICS"
  "bag_topics:=$BAG_TOPICS"
)
printf -v QUOTED_LAUNCH_ARGS '%q ' "${LAUNCH_ARGS[@]}"

echo "=== Stopping any previous mapping session ==="
tmux kill-session -t $SESSION 2>/dev/null || true
pkill -f online_node           2>/dev/null || true
pkill -f online_imu_rate_node  2>/dev/null || true
pkill -f velodyne_driver_node  2>/dev/null || true
pkill -f velodyne_transform_node 2>/dev/null || true
pkill -f velodyne_laserscan_node 2>/dev/null || true
pkill -f mavros_node           2>/dev/null || true
pkill -f imu_repub             2>/dev/null || true
pkill -f rviz2                 2>/dev/null || true
pkill -f static_transform_publisher 2>/dev/null || true
sleep 2

echo "=== Resetting eno1 for LiDAR link ==="
sudo ip link set eno1 down 2>/dev/null; sleep 1
sudo ip link set eno1 up   2>/dev/null; sleep 2
sudo ip addr add 192.168.8.100/24 dev eno1 2>/dev/null || true

echo -n "Checking LiDAR (192.168.8.201) via ping/UDP... "
if ping -c 1 -W 2 192.168.8.201 > /dev/null 2>&1; then
    echo "OK (ping)"
else
    # Velodyne may ignore ping but still stream UDP — check port 2368
    _probe_log=$(mktemp)
    timeout 3 nc -u -l -p 2368 -v > "$_probe_log" 2>&1 || true
    if grep -q "Connection received on 192.168.8.201" "$_probe_log"; then
        echo "OK (UDP)"
    else
        echo "WARNING: no ping or UDP response — continuing anyway (check cable/power if no data)"
    fi
    rm -f "$_probe_log"
fi

echo "=== Starting sensors + RKO-LIO (tmux:$SESSION/slam) ==="
tmux new-session -d -s $SESSION -n slam -x 220 -y 50 \
  "bash -c '$ROS_SETUP && ros2 launch turtlebot_rkolio_hardware hardware_rkolio.launch.py $QUOTED_LAUNCH_ARGS; exec bash'"

tmux select-window -t $SESSION:slam

echo ""
echo "┌─────────────────────────────────────────────────────┐"
echo "│  Mapping started in tmux session: $SESSION           │"
echo "│  slam   window → sensors + RKO-LIO + RViz           │"
if [ "$RECORD_BAG" = "true" ]; then
echo "│  rosbag → starts in ${BAG_START_DELAY}s → $BAG_OUTPUT_DIR"
else
echo "│  rosbag → not started                                │"
echo "│  Record later: bash .../scripts/record_bag.sh        │"
fi
echo "│                                                      │"
echo "│  Attach:    tmux attach -t $SESSION                  │"
echo "│  Stop all:  tmux kill-session -t $SESSION            │"
echo "└─────────────────────────────────────────────────────┘"
