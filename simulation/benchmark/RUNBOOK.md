# LIO-SAM vs RKO-LIO Simulation Experiment — RUNBOOK

**You drive. Each step has the exact command and what to expect.**

Open separate terminals for each step that runs in foreground. Don't close them until that phase is done.

---

## ⚙️ One-time per terminal — source the workspace

Run this **first in every new terminal** you open:

```bash
source /opt/ros/humble/setup.bash
source ~/tb3_3d_ws/install/setup.bash
```

(Already in your `~/.bashrc`, but safer to repeat.)

---

# PHASE 6 — Record sensor bag (small_house)

Goal: drive ONE loop in small_house and capture raw sensors → one bag we'll replay through both algorithms.

You will need **4 terminals** for this phase. Open them in order.

## 6.1 — Terminal A: launch Gazebo + robot (NO SLAM)

```bash
ros2 launch ~/simulation_experiment/configs/record_sim_only.launch.py world:=small_house
```

**Wait ~30–60s** for Gazebo to load (small_house has 89 AWS models).
**You should see:**
- Gazebo window opens
- Many texture warnings (ignore — cosmetic only)
- Log line: `Successfully spawned entity [custom_turtlebot_rkolio]`
- Then: `Wheel pair 1 separation set to [0.287000m]`

Leave this terminal alone — Gazebo runs here.

## 6.2 — Terminal B: launch RKO-LIO live (for live map view while you drive)

This is **only for visualization** so you can see the map being built while you drive. The bag you record next captures only raw sensors, so the LIO-SAM replay later is still fair.

```bash
ros2 run rko_lio online_node \
  --ros-args \
  --params-file ~/tb3_3d_ws/install/turtlebot_rkolio_sim/share/turtlebot_rkolio_sim/config/sim_rkolio_config.yaml \
  -p use_sim_time:=true \
  -r pointcloud:=/velodyne_points \
  -r imu:=/mavros/imu/data
```

**You should see:**
- `RKO LIO Node is up!`
- `First LiDAR received, using as global frame.`
- `LIO initialized using 4 IMU measurements.`

## 6.3 — Terminal C: launch RViz

```bash
ros2 run rviz2 rviz2 \
  -d ~/tb3_3d_ws/install/turtlebot_rkolio_sim/share/turtlebot_rkolio_sim/config/sim_rviz.rviz \
  --ros-args -p use_sim_time:=true
```

**If the map doesn't appear:** in RViz click `Add` → `By Topic` → `/rko_lio/local_map` → `PointCloud2`. Fixed Frame should be `odom`.

## 6.4 — Terminal D: launch teleop (driving)

```bash
ros2 launch turtlebot_rkolio_sim teleop.launch.py
```

A **new gnome-terminal** opens with `teleop_twist_keyboard`. **Click into it** before typing. Keys:
- `i` forward, `,` back, `j/l` turn left/right, `k` stop
- `u/o/m/.` for diagonal moves
- `q/z` to increase/decrease speed (start slow!)

## 6.5 — Terminal E: START THE BAG RECORDING (this is what you control)

When you're ready to start recording, open Terminal E and run:

```bash
mkdir -p ~/simulation_experiment/bags
cd ~/simulation_experiment/bags

ros2 bag record \
  /velodyne_points \
  /mavros/imu/data \
  /odom \
  /tf \
  /tf_static \
  /clock \
  -o sim_small_house_01
```

**Now drive.** The terminal prints `Recording...` and stays running.

### Driving plan (3–5 minutes)
- **DRIVE SLOWLY.** Slower = less drift. Use `q/z` to set a low speed (~0.15 m/s linear, 0.5 rad/s angular).
- Explore as many rooms as you can.
- **Return near the start position at the end** (closes the loop — important for LIO-SAM).
- Avoid sharp pivots — wide gentle turns are better.

### Stop recording

In Terminal E press **`Ctrl+C`**. You'll see `Closing all open writers.`

## 6.6 — Stop the rest

In each of Terminals B, C, D press `Ctrl+C`. In Terminal A press `Ctrl+C` to close Gazebo.

## 6.7 — Verify the bag

```bash
ros2 bag info ~/simulation_experiment/bags/sim_small_house_01/
```

**You should see:**
- Duration: at least 180s (3 minutes)
- Topics: 6 (velodyne_points, mavros/imu/data, odom, tf, tf_static, clock)
- `/velodyne_points` message count > 1800 (= 10 Hz × 180s)
- `/mavros/imu/data` count > 9000 (= 50 Hz × 180s)

If the bag is too short, repeat phase 6 — delete it first:
```bash
rm -rf ~/simulation_experiment/bags/sim_small_house_01
```

**📸 Take a screenshot** of the `ros2 bag info` output → save as `~/simulation_experiment/screenshots/06_bag_info.png`

---

# PHASE 7 — Replay bag through LIO-SAM

Open 3 terminals.

## 7.1 — Terminal A: launch LIO-SAM only (no Gazebo)

```bash
ros2 launch ~/simulation_experiment/configs/replay_liosam.launch.py
```

**You should see:**
- robot_state_publisher start
- vlp16_ring_time_fixer ready: `/velodyne_points → /velodyne_points_proc`
- LIO-SAM nodes: imuPreintegration, imageProjection, featureExtraction, mapOptimization
- RViz opens (LIO-SAM layout)

## 7.2 — Terminal B: record LIO-SAM output

```bash
cd ~/simulation_experiment/bags
ros2 bag record \
  /lio_sam/mapping/odometry \
  /lio_sam/mapping/cloud_registered \
  -o liosam_output_small_house_01
```

## 7.3 — Terminal C: replay the bag

```bash
ros2 bag play ~/simulation_experiment/bags/sim_small_house_01/ --clock --rate 0.5
```

**Note: `--rate 0.5` plays at half speed** — gives LIO-SAM time to keep up. If your CPU is fast you can try `--rate 1.0`.

Watch RViz: LIO-SAM should build a map as the bag plays. Wait for `Reached end of file` in Terminal C.

## 7.4 — Stop and verify

- In Terminal B: `Ctrl+C` (stops recording)
- In Terminal A: `Ctrl+C` (stops LIO-SAM)

```bash
ros2 bag info ~/simulation_experiment/bags/liosam_output_small_house_01/
```

`/lio_sam/mapping/odometry` should have hundreds–thousands of messages.

**📸 Screenshot** the LIO-SAM map in RViz before closing → `screenshots/07_liosam_map.png`

---

# PHASE 8 — Replay bag through RKO-LIO

Open 3 terminals.

## 8.1 — Terminal A: launch RKO-LIO only

```bash
ros2 launch ~/simulation_experiment/configs/replay_rkolio.launch.py
```

## 8.2 — Terminal B: record RKO-LIO output

```bash
cd ~/simulation_experiment/bags
ros2 bag record \
  /rko_lio/odometry \
  /rko_lio/local_map \
  -o rkolio_output_small_house_01
```

## 8.3 — Terminal C: replay the same bag

```bash
ros2 bag play ~/simulation_experiment/bags/sim_small_house_01/ --clock --rate 0.5
```

Wait until `Reached end of file`.

## 8.4 — Stop and verify

```bash
ros2 bag info ~/simulation_experiment/bags/rkolio_output_small_house_01/
```

**📸 Screenshot** the RKO-LIO map → `screenshots/08_rkolio_map.png`

---

# PHASE 9 — Evaluate with evo

These commands all run in **ONE terminal** (no Gazebo, no SLAM). Make sure evo is in your PATH:

```bash
export PATH=$HOME/.local/bin:$PATH
which evo_ape   # should print /home/anas/.local/bin/evo_ape
```

## 9.1 — APE for LIO-SAM (ground truth = /odom, estimate = LIO-SAM)

```bash
cd ~/simulation_experiment/results

evo_ape bag2 \
  ~/simulation_experiment/bags/sim_small_house_01/ \
  ~/simulation_experiment/bags/liosam_output_small_house_01/ \
  /odom \
  /lio_sam/mapping/odometry \
  --align --correct_scale \
  --plot --plot_mode xy \
  --save_results liosam_ape_small_house.zip \
  --save_plot liosam_ape_small_house.pdf 2>&1 | tee liosam_ape_small_house.txt
```

A plot window opens. **📸 Screenshot** it → `screenshots/09_liosam_ape.png`. Close the plot to continue.

## 9.2 — APE for RKO-LIO

```bash
evo_ape bag2 \
  ~/simulation_experiment/bags/sim_small_house_01/ \
  ~/simulation_experiment/bags/rkolio_output_small_house_01/ \
  /odom \
  /rko_lio/odometry \
  --align --correct_scale \
  --plot --plot_mode xy \
  --save_results rkolio_ape_small_house.zip \
  --save_plot rkolio_ape_small_house.pdf 2>&1 | tee rkolio_ape_small_house.txt
```

**📸 Screenshot** → `screenshots/10_rkolio_ape.png`

## 9.3 — Side-by-side comparison

```bash
evo_res \
  liosam_ape_small_house.zip \
  rkolio_ape_small_house.zip \
  --plot --save_plot comparison_ape_small_house.pdf
```

**📸 Screenshot** the comparison plot → `screenshots/11_comparison_ape.png` — **this is a paper figure.**

## 9.4 — Trajectory overlay (best for the paper)

```bash
evo_traj bag2 \
  ~/simulation_experiment/bags/sim_small_house_01/ \
  ~/simulation_experiment/bags/liosam_output_small_house_01/ \
  ~/simulation_experiment/bags/rkolio_output_small_house_01/ \
  /odom \
  /lio_sam/mapping/odometry \
  /rko_lio/odometry \
  --ref /odom \
  --align --plot --plot_mode xy \
  --save_plot trajectory_overlay_small_house.pdf
```

**📸 Screenshot** → `screenshots/12_trajectory_overlay_PAPER_FIGURE.png` — **THE main paper figure.**

## 9.5 — RPE for both

```bash
# LIO-SAM RPE
evo_rpe bag2 \
  ~/simulation_experiment/bags/sim_small_house_01/ \
  ~/simulation_experiment/bags/liosam_output_small_house_01/ \
  /odom /lio_sam/mapping/odometry \
  --align --delta 1 --delta_unit m \
  --save_results liosam_rpe_small_house.zip 2>&1 | tee liosam_rpe_small_house.txt

# RKO-LIO RPE
evo_rpe bag2 \
  ~/simulation_experiment/bags/sim_small_house_01/ \
  ~/simulation_experiment/bags/rkolio_output_small_house_01/ \
  /odom /rko_lio/odometry \
  --align --delta 1 --delta_unit m \
  --save_results rkolio_rpe_small_house.zip 2>&1 | tee rkolio_rpe_small_house.txt
```

---

# PHASE 11 — Repeat for warehouse_no_roof

Same procedure as Phases 6–9 but with `world:=warehouse_no_roof` and bag names ending `_warehouse_01` instead of `_small_house_01`.

Suggested name pattern:
- `sim_warehouse_01` (raw sensors)
- `liosam_output_warehouse_01`
- `rkolio_output_warehouse_01`

The driving strategy is the same: slow, explore, return to start. The warehouse has long aisles — drive at least one full loop around the shelves.

---

# PHASE 12 — Collect final numbers

```bash
echo "=== HOUSE WORLD ===" > ~/simulation_experiment/results/FINAL_RESULTS.txt
echo "--- LIO-SAM APE ---" >> ~/simulation_experiment/results/FINAL_RESULTS.txt
grep -E "rmse|mean|median|std" ~/simulation_experiment/results/liosam_ape_small_house.txt >> ~/simulation_experiment/results/FINAL_RESULTS.txt

echo "--- RKO-LIO APE ---" >> ~/simulation_experiment/results/FINAL_RESULTS.txt
grep -E "rmse|mean|median|std" ~/simulation_experiment/results/rkolio_ape_small_house.txt >> ~/simulation_experiment/results/FINAL_RESULTS.txt

echo "--- LIO-SAM RPE ---" >> ~/simulation_experiment/results/FINAL_RESULTS.txt
grep -E "rmse|mean|median|std" ~/simulation_experiment/results/liosam_rpe_small_house.txt >> ~/simulation_experiment/results/FINAL_RESULTS.txt

echo "--- RKO-LIO RPE ---" >> ~/simulation_experiment/results/FINAL_RESULTS.txt
grep -E "rmse|mean|median|std" ~/simulation_experiment/results/rkolio_rpe_small_house.txt >> ~/simulation_experiment/results/FINAL_RESULTS.txt

cat ~/simulation_experiment/results/FINAL_RESULTS.txt
```

**📸 Screenshot** → `screenshots/15_final_numbers.png`

---

# Troubleshooting

### "RKO-LIO drift / ghost walls"
- **Drive slower.** Use `q/z` to drop speed in teleop.
- Avoid sharp pivots; do wide arcs.
- Drift is expected in pure LIO without loop closure. The numbers (RMSE) quantify it — that's the paper's job.

### "Gazebo too slow / freezes"
- Close other apps. small_house and bookstore are heavy.
- Try `warehouse_no_roof` instead — only 27 models.

### "LIO-SAM crashes / no output"
- Check Terminal A for an error in `imuPreintegration` or `mapOptimization`.
- The bag must have `/mavros/imu/data` flowing — confirm with `ros2 bag info <bag>`.
- Replay at `--rate 0.3` if your CPU can't keep up at 0.5.

### "evo says topic not found"
- Verify the exact topic name with `ros2 bag info <bag>`. Topic names sometimes have a leading slash, sometimes not.

### "Bag is bigger than 1GB"
- That's fine. small_house bag at 3-4 min should be ~1-2 GB.
- If you're running out of disk: `df -h ~/`. You have ~13 GB free.

### "I want to start a phase over"
- Delete the bag(s) you don't want:
```bash
rm -rf ~/simulation_experiment/bags/<bag_name>
```
- Then re-run that phase from the top.

---

# File map

| What | Where |
|---|---|
| Runbook (this file) | `~/simulation_experiment/RUNBOOK.md` |
| Experiment log | `~/simulation_experiment/EXPERIMENT_LOG.md` |
| Launch files | `~/simulation_experiment/configs/*.launch.py` |
| Bags | `~/simulation_experiment/bags/` |
| evo results | `~/simulation_experiment/results/` |
| Screenshots | `~/simulation_experiment/screenshots/` |
| Logs | `~/simulation_experiment/logs/` |

---

End of runbook. Drive slow. Save your screenshots.
