# Simulation Experiment Log — LIO-SAM vs RKO-LIO
Started: Çrş 20 May 2026 00:39:56 +03
Operator: Anas
Host: anas


## [STEP 0] — System Check
- Status: ✅ DONE
- Time: $(date)

### Environment
- ROS_DISTRO: humble
- Ubuntu: 22.04.5 LTS
- RAM: 31Gi (26Gi free)
- GPU: NVIDIA + CUDA 12.2
- Gazebo: 11.10.2
- GTSAM: ros-humble-gtsam 4.2.0 (apt)
- evo: ❌ NOT INSTALLED — will install
- **Disk: 7.8GB free of 129GB (94% used) — TIGHT, may need cleanup**

### Workspace
- Active workspace: `~/tb3_3d_ws` (NOT `~/ros2_ws` from instructions)
- `~/ros2_ws` only has `tubitak_rl`, ignore
- Built packages confirmed: `lio_sam`, `rko_lio`, `turtlebot_rkolio_sim`, `fast_lio`, `livox_ros_driver2`
- ⚠️ `src/rko_lio/` source files deleted in git (install/ still intact — usable)

### CRITICAL DISCOVERY
The `turtlebot_rkolio_sim` package is **already fully configured** for this exact experiment:
- URDF: custom_turtlebot.urdf.xacro with Jetson + Pixhawk + VLP-16 stack
- Sensors: VLP-16 (16x1800, 10Hz) on `/velodyne_points`, IMU (50Hz) on `/mavros/imu/data`
- 16 worlds: corridor, basement_room, tunnel, house, cafe, warehouse, bookstore, etc.
- Pre-tuned configs: liosam_vlp16_sim.yaml, sim_rkolio_config.yaml, fastlio_vlp16_sim.yaml
- Launch files: simulation.launch.py (Gazebo only), sim_with_{liosam,rkolio,fastlio}.launch.py (full stack)
- Helper scripts: run_sim.sh, record_sim.sh
- `vlp16_ring_time_fixer.py` preprocessor (LIO-SAM needs ring + time fields — Gazebo doesn't emit them)

### Topic mapping
- LIO-SAM consumes `/velodyne_points_proc` (after ring/time fixer) + `/mavros/imu/data`
- RKO-LIO consumes `/velodyne_points` (raw) + `/mavros/imu/data`
- Both use frames: base_link / imu_link / velodyne / odom

### Variance from master plan
- Skipping Phase 1 setup (URDF/dependencies/install LIO-SAM) — already done
- TURTLEBOT3_MODEL=waffle (not waffle_pi) — but we use custom SDF, not stock TB3
- Default world for launchers is `corridor` (master plan starts with house — we can pass either)


## [STEP 5] — Smoke Test (small_house world)
- Status: ✅ DONE
- Time: $(date)

### Launch command tested
```
ros2 launch ~/simulation_experiment/configs/record_sim_only.launch.py world:=small_house
```

### Topics verified publishing
- `/velodyne_points`: 14,374 points/scan, frame=velodyne, point_step=16 (raw x/y/z/intensity)
- `/mavros/imu/data`: 50Hz, frame=imu_link, orientation OK
- `/odom`: published by gazebo_ros_diff_drive, frame=odom
- `/tf` + `/tf_static`: present
- `/clock`: present

### TF tree spot-check
- base_link → velodyne: translation [-0.032, 0.000, 0.165], identity rotation ✅

### Known cosmetic issues (don't affect SLAM)
- AWS RoboMaker model texture paths broken (Portrait*.jpg) — material warnings in gzclient.
  Doesn't affect physics or sensor data. Pure visual.

### Standalone launchers ready in ~/simulation_experiment/configs/
- record_sim_only.launch.py — Gazebo + robot (no SLAM) — used for bag recording
- replay_liosam.launch.py    — LIO-SAM nodes + ring/time fixer + RViz (no Gazebo) — for bag replay
- replay_rkolio.launch.py    — RKO-LIO node + RViz (no Gazebo) — for bag replay


## [STEP 6] — Bag Recording (small_house)
- Status: ⏳ IN PROGRESS
- Started: $(date)
- Bag path: ~/simulation_experiment/bags/sim_small_house_01
- Topics recorded: /velodyne_points, /mavros/imu/data, /odom, /tf, /tf_static, /clock
- Gazebo backgrounded (bul5r0b88), bag recorder (bcx1i0vda), teleop in gnome-terminal


## [STEP 6 attempt 1] — Aborted
- Status: ❌ ABORTED at $(date)
- Reason: Switching to user-driven control. Recording auto-start was confusing.
- Action: Killed all sim processes, deleted partial bag (~600MB into recording).
- Drift observation in screenshot: real — expected for RKO-LIO without loop closure. User noted slower driving will help.

## Runbook created
- Location: ~/simulation_experiment/RUNBOOK.md
- Full step-by-step playbook with copy-paste commands for phases 6 → 12.
- User now drives the experiment themselves.

