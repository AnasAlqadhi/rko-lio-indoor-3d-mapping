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

if [ ! -d "$BAG_PATH" ] || [ -z "$(ls -A $BAG_PATH 2>/dev/null)" ]; then
    echo "ERROR: No recordings found in $BAG_PATH"
    exit 1
fi

folders=("$BAG_PATH"/*/)

echo "---------------------------------------------"
echo "Available recordings:"
echo "---------------------------------------------"
for i in "${!folders[@]}"; do
    echo "[$i] $(basename "${folders[$i]}")"
done
echo "---------------------------------------------"
read -r -p "Enter the recording number to play: " choice

if ! [[ "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -ge "${#folders[@]}" ]; then
    echo "Invalid selection"
    exit 1
fi

selected_bag="${folders[$choice]}"
echo "---------------------------------------------"
echo "PLAYING BAG: $selected_bag"
echo "Pause/Resume: SPACE"
echo "Rate control: Arrow Up / Arrow Down"
echo "---------------------------------------------"

bash "$HOME/tb3_3d_ws/src/turtlebot_rkolio_hardware/scripts/run_rko_playback.sh" "$selected_bag"
