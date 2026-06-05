#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
#  Full mapping session recorder — captures raw sensors + all pose outputs
#  from both RKO-LIO and LIO-SAM (whichever is running).
#
#  Usage: ./record_full.sh [session_name]
#  Output: ~/tb3_3d_ws/recordings/<session_name>_<timestamp>/
#
#  On Ctrl+C: saves LIO-SAM map (PCD + poses) automatically if running.
# ─────────────────────────────────────────────────────────────────────────────

SESSION=${1:-"mapping"}
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_DIR="$HOME/tb3_3d_ws/recordings/${SESSION}_${TIMESTAMP}"
MAP_DIR="$HOME/tb3_3d_ws/recordings/${SESSION}_${TIMESTAMP}_map"

mkdir -p "$BAG_DIR"

source /opt/ros/humble/setup.bash
source ~/tb3_3d_ws/install/setup.bash
export ROS_DOMAIN_ID=45

echo "=== Full Mapping Recorder ==="
echo "  Session: $SESSION"
echo "  Bag:     $BAG_DIR"
echo "  Map:     $MAP_DIR (saved on exit if LIO-SAM is running)"
echo ""

# ── On exit: save LIO-SAM map if the service exists ─────────────────────────
save_liosam_map() {
    echo ""
    echo "=== Saving LIO-SAM map before exit... ==="
    mkdir -p "$MAP_DIR"
    if ros2 service list 2>/dev/null | grep -q "/lio_sam/save_map"; then
        ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \
            "{resolution: 0.2, destination: '$MAP_DIR'}" && \
            echo "  Map saved to $MAP_DIR" || \
            echo "  WARNING: LIO-SAM save_map call failed"
    else
        echo "  INFO: /lio_sam/save_map service not found — skipping map save"
    fi
}
trap save_liosam_map EXIT

# ── Verify that at least one SLAM output is present ─────────────────────────
echo "Waiting up to 15 s for SLAM topics..."
for i in $(seq 1 15); do
    if ros2 topic list 2>/dev/null | grep -qE "/rko_lio/odometry|/lio_sam/mapping/odometry"; then
        echo "  SLAM topics detected."
        break
    fi
    sleep 1
    if [ "$i" -eq 15 ]; then
        echo "  WARNING: No SLAM odometry topic found after 15 s."
        echo "  Recording raw sensors only (start your SLAM algorithm)."
    fi
done

echo ""
echo "Recording... Press Ctrl+C to stop and save map."
echo ""

# ── Record ───────────────────────────────────────────────────────────────────
# Raw sensors
SENSOR_TOPICS=(
    /velodyne_points
    /mavros/imu/data
    /mavros/imu/data_raw
    /tf
    /tf_static
)

# RKO-LIO output
RKOLIO_TOPICS=(
    /rko_lio/odometry
    /rko_lio/local_map
    /rko_lio/path
    /rko_lio/deskewed_scan
)

# LIO-SAM output
LIOSAM_TOPICS=(
    /lio_sam/mapping/odometry
    /lio_sam/mapping/odometry_incremental
    /lio_sam/mapping/cloud_registered
    /lio_sam/mapping/map_global
    /lio_sam/mapping/path
    /lio_sam/imu/path
    /lio_sam/imu/odometry
)

ros2 bag record \
    -o "$BAG_DIR" \
    --include-hidden-topics \
    "${SENSOR_TOPICS[@]}" \
    "${RKOLIO_TOPICS[@]}" \
    "${LIOSAM_TOPICS[@]}"
