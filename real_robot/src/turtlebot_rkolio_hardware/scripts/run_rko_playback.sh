#!/bin/bash

set -e

if [ $# -ne 1 ]; then
    echo "Usage: $0 <bag_directory>"
    exit 1
fi

BAG_PATH="$1"

if [ ! -d "$BAG_PATH" ]; then
    echo "ERROR: Bag directory not found: $BAG_PATH"
    exit 1
fi

source /opt/ros/humble/setup.bash
source "$HOME/tb3_3d_ws/install/setup.bash"
export ROS_DOMAIN_ID=45

CONFIG_FILE="$HOME/tb3_3d_ws/src/turtlebot_rkolio_hardware/config/rkolio_params.yaml"
RVIZ_CONFIG="$HOME/tb3_3d_ws/src/turtlebot_rkolio_hardware/config/mapping_view.rviz"

if tmux has-session -t mapping 2>/dev/null; then
    echo "ERROR: Live mapping tmux session is running."
    echo "Stop it first with: tmux kill-session -t mapping"
    exit 1
fi

# When launched over SSH, restore the physical desktop display if available.
# When launched from VNC, DISPLAY is already set and RViz will stay there.
if [ -z "$DISPLAY" ] && pgrep -u "$(id -u)" -f "/usr/bin/gnome-shell" >/dev/null 2>&1; then
  export DISPLAY=:0
  export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
  export XDG_RUNTIME_DIR="/run/user/$(id -u)"
  export DBUS_SESSION_BUS_ADDRESS="unix:path=${XDG_RUNTIME_DIR}/bus"
fi

cleanup() {
    if [ -n "$RVIZ_PID" ]; then
        kill "$RVIZ_PID" 2>/dev/null || true
    fi
    if [ -n "$RKO_PID" ]; then
        kill "$RKO_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

echo "Starting offline RKO-LIO playback stack..."
echo "Bag: $BAG_PATH"

ros2 run rko_lio online_node \
    --ros-args \
    -r __node:=rko_lio_playback \
    --params-file "$CONFIG_FILE" \
    -p use_sim_time:=true \
    >/tmp/rko_playback_node.log 2>&1 &
RKO_PID=$!

RVIZ_PID=""
if [ -n "$DISPLAY" ]; then
    rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true \
        >/tmp/rko_playback_rviz.log 2>&1 &
    RVIZ_PID=$!
else
    echo "INFO: DISPLAY is not set, so RViz was not started."
fi

sleep 5

echo "Starting rosbag playback with /clock..."
ros2 bag play "$BAG_PATH" --clock