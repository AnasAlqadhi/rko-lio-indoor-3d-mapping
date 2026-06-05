#!/bin/bash
# Start complete dual robot system with live merging

echo "========================================="
echo "Starting Dual Robot Mapping System"  
echo "========================================="

# Step 1: Start local robot system
echo "Starting local robot system..."
~/tb3_3d_ws/scripts/complete_robot_startup.sh &
LOCAL_PID=$!

sleep 15

# Step 2: Wait for drone data over network
echo "Waiting for drone data over network..."
echo "Make sure drone system is running with ROS_DOMAIN_ID=45"

timeout 30s bash -c 'until ros2 topic list | grep -q rko_lio; do sleep 2; done'

if [ $? -eq 0 ]; then
    echo "✓ Drone data detected over network"
else
    echo "⚠ Continuing without drone data - will merge when available"
fi

# Step 3: Start live map merger
echo "Starting live map merger..."
python3 ~/tb3_3d_ws/scripts/live_map_merger.py &
MERGER_PID=$!

# Step 4: Start live recording (optional)
read -p "Start live recording of merged session? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    ~/tb3_3d_ws/scripts/live_recording.sh &
    RECORD_PID=$!
    echo "✓ Live recording started"
fi

echo "========================================="
echo "Dual robot system is now running!"
echo "Monitor topics:"
echo "  /rko_lio/local_map (local robot)"  
echo "  /live_merged_map (combined map)"
echo "Press Ctrl+C to stop all processes"
echo "========================================="

# Cleanup on exit
trap "kill $MERGER_PID $RECORD_PID $LOCAL_PID 2>/dev/null; exit" SIGINT
wait
