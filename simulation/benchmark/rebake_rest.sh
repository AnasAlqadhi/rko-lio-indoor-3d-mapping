#!/bin/bash
# rebake_rest.sh — pre-bake + FAST-LIO replay for warehouse + bookstore, so all
# three worlds use the drop-free baked pipeline. Deletes each proc bag after use
# to stay within disk. Results overwrite fastlio_*_{warehouse,bookstore} txt/zip.
set -e
cd ~/simulation_experiment
BAGS=bags
source /opt/ros/humble/setup.bash
source ~/tb3_3d_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export PATH=$HOME/.local/bin:$PATH

do_world() {
    local W="$1" TAG="$2"
    echo "=================== RE-BAKE $W ==================="
    rm -rf "$BAGS/sim_${W}_proc" "$BAGS/fastlio_baked_${W}"
    python3 bake_proc_bag.py "$BAGS/sim_$W" "$BAGS/sim_${W}_proc"
    ros2 bag reindex "$BAGS/sim_${W}_proc" 2>/dev/null || true

    ros2 run fast_lio fastlio_mapping --ros-args \
        --params-file ~/tb3_3d_ws/src/turtlebot_rkolio_sim/config/fastlio_vlp16_sim.yaml \
        -p use_sim_time:=true > logs/rebake_${W}_node.log 2>&1 &
    sleep 8
    ros2 bag record /Odometry -o "$BAGS/fastlio_baked_${W}" > logs/rebake_${W}_rec.log 2>&1 &
    sleep 3
    ros2 bag play "$BAGS/sim_${W}_proc" --clock --rate 1.0 \
        --topics /velodyne_points_proc /mavros/imu/data > logs/rebake_${W}_play.log 2>&1
    sleep 4
    pkill -INT -f "bag record"; sleep 3
    for p in $(pgrep -f fastlio_mapping) $(pgrep -f 'bag record'); do kill -9 $p 2>/dev/null || true; done
    sleep 2

    [ ! -f "$BAGS/fastlio_baked_${W}/metadata.yaml" ] && ros2 bag reindex "$BAGS/fastlio_baked_${W}" 2>/dev/null || true

    # merge with GT and evaluate
    rm -rf "$BAGS/fastlio_baked_merged_${W}"
    python3 merge_bags.py "$BAGS/sim_$W" "$BAGS/fastlio_baked_${W}" "$BAGS/fastlio_baked_merged_${W}"
    sed -i "s/storage_identifier: ''/storage_identifier: sqlite3/" "$BAGS/fastlio_baked_merged_${W}/metadata.yaml" 2>/dev/null || true
    sed -i 's/storage_identifier: ""/storage_identifier: sqlite3/' "$BAGS/fastlio_baked_merged_${W}/metadata.yaml" 2>/dev/null || true

    echo "--- $TAG APE (baked) ---"
    evo_ape bag2 "$BAGS/fastlio_baked_merged_${W}/" /odom /Odometry --align --correct_scale \
        --save_results results/fastlio_ape_${TAG}.zip 2>&1 | grep -iE "rmse|mean|median|std|max|min" | tee results/fastlio_ape_${TAG}.txt
    echo "--- $TAG RPE (baked) ---"
    evo_rpe bag2 "$BAGS/fastlio_baked_merged_${W}/" /odom /Odometry --align --delta 1 --delta_unit m \
        --save_results results/fastlio_rpe_${TAG}.zip 2>&1 | grep -iE "rmse|mean|median" | tee results/fastlio_rpe_${TAG}.txt

    # refresh the merged bag used by plots, then reclaim disk
    rm -rf "$BAGS/fastlio_merged_${W}" && cp -r "$BAGS/fastlio_baked_merged_${W}" "$BAGS/fastlio_merged_${W}"
    rm -rf "$BAGS/sim_${W}_proc"     # reclaim the big proc bag
    echo "[$W] baked loop-back=$(grep -c 'loop back' logs/rebake_${W}_node.log) No-Eff=$(grep -c 'No Effective Points' logs/rebake_${W}_node.log)"
}

do_world warehouse_06 warehouse
do_world bookstore_01 bookstore

echo "=================== RE-BAKE COMPLETE ==================="
echo "Regenerate table+figures with: python3 build_stats_table.py && python3 plot_multipanel.py"
