#!/bin/bash

if [ $# -eq 0 ]; then
    echo "Usage: $0 <recording_name>"
    echo "Available recordings:"
    ls -la ~/tb3_3d_ws/recordings/
    exit 1
fi

RECORDING_PATH="$HOME/tb3_3d_ws/recordings/$1"

if [ ! -d "$RECORDING_PATH" ]; then
    echo "Error: Recording not found: $RECORDING_PATH"
    exit 1
fi

echo "Playing back recording: $1"
bash "$HOME/tb3_3d_ws/src/turtlebot_rkolio_hardware/scripts/run_rko_playback.sh" "$RECORDING_PATH"
