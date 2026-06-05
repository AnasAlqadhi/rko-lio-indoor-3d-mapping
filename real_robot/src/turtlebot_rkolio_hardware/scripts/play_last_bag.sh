#!/bin/bash

source /opt/ros/humble/setup.bash
source "$HOME/tb3_3d_ws/install/setup.bash"
export ROS_DOMAIN_ID=45

SSD=/mnt/ssd
BAG_PATH=$SSD/recordings

# Auto-mount SSD if needed
if ! mountpoint -q $SSD; then
  echo "=== Mounting SSD ==="
  sudo mount /dev/nvme0n1p1 $SSD
  if ! mountpoint -q $SSD; then
    echo "ERROR: Could not mount SSD."
    exit 1
  fi
fi

latest_bag=$(ls -td "$BAG_PATH"/*/ 2>/dev/null | head -1)

if [ -z "$latest_bag" ]; then
    echo "ERROR: No recordings found in $BAG_PATH"
    exit 1
fi

echo "----------------------------------------------------"
echo "PLAYING LATEST BAG: $latest_bag"
echo "Pause/Resume: SPACE"
echo "Rate control: Arrow Up / Arrow Down"
echo "----------------------------------------------------"

bash "$HOME/tb3_3d_ws/src/turtlebot_rkolio_hardware/scripts/run_rko_playback.sh" "$latest_bag"
