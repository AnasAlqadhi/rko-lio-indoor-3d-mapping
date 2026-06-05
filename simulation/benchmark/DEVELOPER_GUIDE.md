# LIO-SAM vs RKO-LIO Simulation Experiment — Developer Guide

A knowledge-transfer document for anyone repeating or extending this experiment. Written from real bring-up experience on Anas's Ubuntu 22.04 / ROS 2 Humble system, May 2026.

If you only want to **run** the experiment, read [`RUNBOOK.md`](RUNBOOK.md).
This file explains **why** every choice was made.

---

## 1. Project context

This experiment exists to support a conference paper (ICHORA 2026, Paper ID 331) titled *Drift-Resilient Indoor 3D Mapping*. The paper compares two LiDAR-Inertial Odometry algorithms in GPS-denied indoor environments:

- **LIO-SAM** — factor-graph LIO with loop closure (Shan et al., 2020)
- **RKO-LIO** — robust kinematic odometry LIO from PRBonn (Vizzo, Guadagnino et al., 2024)

The original paper had real-world data only. We added **simulated experiments** in this work to:
1. Have a controlled environment with ground-truth pose (Gazebo `/odom`)
2. Compare algorithms on identical sensor data (impossible with real hardware)
3. Produce reproducible results for the paper

A third algorithm — **FAST-LIO** — is built and available in the workspace but was scoped out of this experiment.

---

## 2. System requirements and dependencies

### Verified working environment

| Component | Version |
|---|---|
| OS | Ubuntu 22.04.5 LTS (Jammy) |
| ROS distribution | ROS 2 Humble Hawksbill |
| Gazebo | Gazebo Classic 11.10.2 |
| Python | 3.10 |
| GPU (optional) | NVIDIA driver 535, CUDA 12.2 |
| RAM | 31 GiB (16+ GiB recommended; Gazebo + LIO-SAM + RViz is heavy) |
| Disk | At least 15 GiB free. A 4-minute bag at full sensor rate is ~1.5 GiB; you need 3× (raw + 2 outputs). |

### Required apt packages

```bash
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-simulations \
  ros-humble-turtlebot3-gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-velodyne-simulator \
  ros-humble-velodyne-description \
  ros-humble-velodyne-gazebo-plugins \
  ros-humble-perception-pcl \
  ros-humble-pcl-msgs \
  ros-humble-xacro \
  ros-humble-gtsam \
  ros-humble-aws-robomaker-small-warehouse-world
```

`ros-humble-gtsam 4.2.0` is what LIO-SAM links against in this workspace. The classic GTSAM PPA (`ppa:borglab/gtsam-release-4.1`) is **not needed** when using the ROS-packaged GTSAM.

### Python tools

```bash
pip3 install --user evo
export PATH=$HOME/.local/bin:$PATH
```

`evo 1.36+` works. The `rosbags` Python package is pulled in as an evo dependency and is what enables `evo_ape bag2` / `evo_rpe bag2` to read ROS 2 sqlite bags directly.

### Worktree layout (this project)

```
~/tb3_3d_ws/
├── src/
│   ├── turtlebot_rkolio_sim/      # The simulation package — owns model, worlds, launchers
│   ├── turtlebot_rkolio_hardware/ # Real-robot launch (not used here)
│   ├── LIO-SAM/                   # https://github.com/TixiaoShan/LIO-SAM @ ros2 branch
│   ├── FAST_LIO/                  # https://github.com/hku-mars/FAST_LIO
│   ├── Bonxai/, Sophus/, Livox-SDK2/, livox_ros_driver2/
│   └── (rko_lio sources were present and built, then deleted on disk — install/ kept)
├── build/  install/  log/
```

The `rko_lio` ROS package is built and lives in `install/`. Its source directory was deleted but the launchers, configs, and binary node remain functional. **Don't delete `install/rko_lio/`.**

---

## 3. The simulated robot

This isn't a stock TurtleBot3. It's a **custom Waffle Pi** with three real-hardware payloads modeled in:

```
base_footprint
└── base_link
    ├── wheel_left_link, wheel_right_link (continuous)
    ├── caster_back_left_link, caster_back_right_link (fixed)
    ├── jetson_housing_link       (Jetson AGX Orin, 110×110×72 mm)
    │   └── velodyne              (VLP-16 cylinder on top)
    └── pixhawk_cube_link         (Pixhawk Cube Orange+, 38×38×22 mm)
        └── imu_link              (IMU is inside the cube)
```

Defined in:
- `src/turtlebot_rkolio_sim/urdf/custom_turtlebot.urdf.xacro` — TF tree for `robot_state_publisher`
- `src/turtlebot_rkolio_sim/models/custom_turtlebot_rkolio/model.sdf` — Gazebo physics, sensors, plugins

**Important:** these two files must stay consistent. The SDF is what Gazebo spawns; the URDF is what publishes TF for SLAM consumers. The standalone `record_sim_only.launch.py` uses the **custom URDF**, not the stock TurtleBot3 URDF (which would put `velodyne` and `imu_link` in the wrong place and silently break SLAM).

### Sensor specifications

| Sensor | Topic | Rate | Frame | Plugin |
|---|---|---|---|---|
| LiDAR (VLP-16) | `/velodyne_points` | 10 Hz | `velodyne` | `gazebo_ros_velodyne_laser` |
| IMU (Pixhawk) | `/mavros/imu/data` | 50 Hz | `imu_link` | `gazebo_ros_imu_sensor` |
| Wheel odom (GT) | `/odom` | 30 Hz | `odom` → `base_footprint` | `gazebo_ros_diff_drive` |

VLP-16 scan configuration (from `sensors.urdf.xacro`):
- 16 vertical beams, ±15° fan, 2° spacing
- 1800 horizontal samples per scan → 14,400 points per scan returned (Gazebo culls some)
- Range: 0.5–50.0 m, σ = 0.01 m Gaussian noise

IMU noise model (50 Hz Pixhawk Cube Orange+):
- Gyro: σ = 0.0005 rad/s (per axis)
- Accel: σ = 0.02 m/s² (per axis)

The `/mavros/imu/data` topic name is intentional — it mirrors the real-robot Pixhawk→MAVROS bridge so the SLAM configs work identically in sim and on hardware.

---

## 4. Important quirks of the Gazebo VLP-16 plugin

> The `gazebo_ros_velodyne_laser` plugin publishes a `sensor_msgs/PointCloud2` with **only** the `x`, `y`, `z`, `intensity` fields.

Real Velodyne drivers publish **two more fields**: `ring` (uint16, which beam the point came from) and `time` (float32, per-point time offset within the scan). LiDAR SLAM that does **deskewing** needs these. The consequences:

- **LIO-SAM** requires `ring` and `time`. Without them it warns and discards the scan.
- **FAST-LIO** also requires them.
- **RKO-LIO** does not require them. It can operate without per-point timestamps (it just turns deskew off — see [`sim_rkolio_config.yaml`](../tb3_3d_ws/src/turtlebot_rkolio_sim/config/sim_rkolio_config.yaml) line `deskew: false`).

### The workaround: `vlp16_ring_time_fixer.py`

Located at `src/turtlebot_rkolio_sim/scripts/vlp16_ring_time_fixer.py`. This node subscribes to `/velodyne_points` and republishes on `/velodyne_points_proc` with `ring` and `time` fields injected:

- `ring`: computed from elevation angle (`atan2(z, sqrt(x²+y²))`) → nearest VLP-16 beam (0..15)
- `time`: computed from azimuth (`atan2(-y, x)`) → fraction of 0.1 s scan period

This is a **synthetic** reconstruction — it is not the same as real per-point timestamps, but it's enough to satisfy LIO-SAM's parser and produce reasonable scan registration.

> **Operational implication:** LIO-SAM launchers (and the `replay_liosam.launch.py` we wrote) always include this node. The LIO-SAM config consumes `/velodyne_points_proc`, not `/velodyne_points`. RKO-LIO consumes `/velodyne_points` directly.

---

## 5. The three SLAM stacks

### 5.1 LIO-SAM (`lio_sam`)

Source: https://github.com/TixiaoShan/LIO-SAM, `ros2` branch.

**Four nodes** (run as separate processes):
- `lio_sam_imuPreintegration` — integrates IMU between scans
- `lio_sam_imageProjection` — converts PointCloud2 → range image, deskews
- `lio_sam_featureExtraction` — LOAM edge/surface features
- `lio_sam_mapOptimization` — GTSAM factor graph + loop closure

**Critical params in `liosam_vlp16_sim.yaml`:**

| Param | Value | Why |
|---|---|---|
| `imuFrequence` | 50 | Pixhawk rate. LIO-SAM was designed for 200 Hz, so... |
| `imuAccNoise` | 0.02 | …we **boost noise covariances** to lower IMU trust at this rate. |
| `imuGyrNoise` | 0.005 | Same reason. |
| `imuAccBiasN` | 0.001 | |
| `imuGyrBiasN` | 0.0001 | |
| `extrinsicTrans` | `[-0.06, 0.0, 0.053]` | IMU (pixhawk) → LiDAR (velodyne) translation in meters. **Sign matters.** |
| `pointCloudTopic` | `/velodyne_points_proc` | After ring/time fixer, not raw. |
| `imuTopic` | `/mavros/imu/data` | Matches real-robot bridge. |
| `sensor` | `velodyne` | Tells LIO-SAM the projection model. |
| `N_SCAN` | 16 | VLP-16. |
| `Horizon_SCAN` | 1800 | Matches the Gazebo plugin's `samples`. |
| `z_tollerance` | 0.3 | TurtleBot is planar; this drops large z jumps. |
| `rotation_tollerance` | 1000.0 | Effectively off — pure xy/yaw motion. |
| `loopClosureEnableFlag` | true | Loop closure on — that's the headline feature. |

#### Common LIO-SAM failure modes

- **Empty scans every frame** → ring/time fixer is not running, or topic name mismatch.
- **Initialization fails ("waiting for IMU")** → `imuFrequence` mismatch with actual rate, or IMU and LiDAR frames are not transformable via TF.
- **Map looks "jumpy" on turns** → IMU bias estimation drifting. Try raising `imuRPYWeight` or lowering speed.
- **Crash in `mapOptimization`** → usually a GTSAM ABI mismatch (e.g. mixing PPA 4.1 with apt 4.2). Stick to one source.

### 5.2 RKO-LIO (`rko_lio`)

Source: github.com/PRBonn/kiss-icp-related — robust LIO from the kiss-icp family.

**One node**: `online_node` does everything.

**Critical params in `sim_rkolio_config.yaml`:**

| Param | Value | Why |
|---|---|---|
| `lidar_topic` | `/velodyne_points` | Raw, no fixer needed. |
| `imu_topic` | `/mavros/imu/data` | Same as LIO-SAM. |
| `deskew` | **false** | Gazebo plugin has no per-point times. Don't lie to the algorithm. |
| `initialization_phase` | true (sim) / **false** (real) | On real hardware there's an SO3::exp crash with this enabled — see commit `382dce4`. Sim is fine with it on. |
| `voxel_size` | 0.3 | Map voxel size. Smaller = denser map = more CPU. |
| `max_range` | 20.0 | Indoor cap. |
| `min_range` | 0.9 | Drops self-returns. |
| `min_beta` | 150.0 | Damping factor for the kinematic model — tuned for 50 Hz IMU. |
| `max_expected_jerk` | 5.0 | Outlier rejection. |
| `publish_local_map` | true | What you see in RViz. |

#### RKO-LIO's character

- No loop closure. Pure odometry. Drift accumulates over the run.
- Very **robust** — won't crash on degenerate scans, IMU dropouts, or fast motion. Quietly degrades.
- Faster than LIO-SAM (single-threaded efficient).
- Map looks "ghosty" on revisits because second-pass scans don't snap to the first pass — see Section 9.

### 5.3 FAST-LIO (`fast_lio`)

Built and available; launchable via `sim_with_fastlio.launch.py`. Same sensor topics as LIO-SAM (consumes `/velodyne_points_proc` from the ring/time fixer + `/mavros/imu/data`). Publishes odometry on **`/Odometry`** in the **`camera_init`** world frame.

**FAST-LIO is now part of the comparison.** Getting it working took real debugging — see below.

#### The `/clock` double-source bug (the big one)

FAST-LIO initially failed on every replay: it produced odometry but the trajectory diverged to ~4 m APE, and the node log was full of:

```
lidar loop back, clear buffer
No Effective Points!
[pcl::VoxelGrid::applyFilter] Leaf size is too small ... indices would overflow
```

Root cause was **not** in FAST-LIO or its config — it was the **replay method**. The sim bags were recorded *with* a `/clock` topic (count == velodyne count). The original replay scripts play that recorded `/clock` **and** pass the `--clock` flag, so rosbag2 generates a second clock. Two competing `/clock` sources make `use_sim_time` jump backward intermittently. FAST-LIO's `standard_pcl_cbk` treats any backward `header.stamp` as a loop-back and **clears its lidar buffer every scan** → it can never accumulate a scan+IMU window → registration fails on every frame.

LIO-SAM and RKO-LIO tolerate the double-clock (they don't hard-reset on backward time), which is why only FAST-LIO exposed the bug.

**Fix:** on replay, exclude the bag's own `/clock` so `--clock` is the single source:

```bash
ros2 bag play <bag> --clock --rate 0.5 --topics /velodyne_points /mavros/imu/data
#                                       ^ note: /clock deliberately NOT listed
```

With the clean clock: loop-backs 646 → **0**, "No Effective Points" 2214 → **0**. See `replay_fastlio.sh`.

#### timestamp_unit must stay `2` (US) — counter-intuitive but deliberate

`fastlio_vlp16_sim.yaml` sets `preprocess.timestamp_unit: 2` (microseconds) even though the fixer emits the per-point `time` field in **seconds**. This is on purpose. The fixer derives per-point time from **azimuth**, which is *not monotonic with point order*; feeding those as real times to FAST-LIO's deskew produces "lidar loop back" / "No Effective Points" / VoxelGrid overflow. Setting `US` scales the times to ~0, neutralising deskew — matching RKO-LIO's `deskew: false`. The sim has no reliable per-point timestamps, so **all three SLAMs run without motion compensation**, which keeps the comparison fair. (Changing this to `0`/SEC re-breaks FAST-LIO.)

#### The Python fixer is a real-time bottleneck on heavy worlds

`vlp16_ring_time_fixer.py` does per-point numpy on ~14k points/scan. On dense worlds (small_house, 89 models) it cannot sustain 0.5× live, which can drop/jitter scans. The robust workaround is to **pre-bake** the proc cloud offline: `bake_proc_bag.py` reads a sim bag and writes a `sim_<world>_proc` bag containing `/velodyne_points_proc` + passthrough `/mavros/imu/data` + `/odom`, with zero dropped scans (no real-time pressure). Replay the baked bag straight into FAST-LIO — no live fixer needed, so it easily sustains rate.

#### Results (APE / RPE RMSE, m)

| World | APE LIO-SAM | APE RKO-LIO | APE FAST-LIO |
|---|---|---|---|
| small_house | 0.049 | 0.148 | 0.465* |
| warehouse | 0.040 | 0.163 | **0.034** |
| bookstore | **21.69** (diverged) | 0.494 | **0.039** |

\* small_house FAST-LIO was run with the live fixer (CPU-bound); re-run with the pre-baked proc bag for the trustworthy number.

Headline: **LIO-SAM is most accurate when it works but catastrophically diverges on bookstore (21.7 m)** — the paper's "LIO-SAM goes crazy" case. **RKO-LIO never diverges** (the robustness thesis). **FAST-LIO is excellent on the open worlds** (warehouse, bookstore) once the clock bug is fixed.

---

## 6. The simulation worlds

`src/turtlebot_rkolio_sim/worlds/` ships 16 worlds. Performance and realism vary:

| World | Models | Size | Description | Best for |
|---|---|---|---|---|
| `corridor` | 2 | 9 KB | Bare box-walled corridor, custom-built | Old: too visually empty for "realistic" testing |
| `tunnel` | 2 | 13 KB | Bare box tunnel | Same critique |
| `basement_room` | custom | 12 KB | Custom basement layout | Light testing |
| `static_arena` | custom | 15 KB | Custom open arena | Light testing |
| `house` | TB3 | 1.4 KB | Stock TB3 house | Underwhelming visually |
| `dqn_stage1..4` | TB3 | 1.5–2 KB | TB3 DRL stages | Not for SLAM |
| `cafe` | 1 AWS bundle | 3.8 KB | Small cafe with tables/chairs | Light realistic |
| `warehouse` | 27 AWS | 7.6 KB | Industrial shelves + roof | Realistic, lit interior |
| `warehouse_no_roof` | 27 AWS | 7.9 KB | Same minus roof | Brighter, more "tunnel-aisle" feel |
| `small_house` | 89 AWS | 29 KB | Furnished residential | **Paper-grade indoor scene** |
| `bookstore` | 141 AWS | 36 KB | Retail with books, shelves | Heaviest. Visually richest. |
| `turtlebot3_world` | TB3 | 1.7 KB | TB3 default test | Quick smoke |
| `willowgarage` | stock | 278 B | Reference to gazebo stock | Skip — heavy and fragile |

### Picking a world for the paper

We chose **three environments** for the ICHORA 2026 comparison:

| # | World | Status | Character |
|---|---|---|---|
| 1 | `small_house` | ✅ **DONE** | Furnished residential, 89 AWS models, multi-room |
| 2 | `warehouse` | ✅ **DONE** | Industrial shelves + roof, 27 AWS models, repetitive geometry |
| 3 | `bookstore` | ✅ **DONE** | Dense retail with books/shelves, 141 models (heaviest) |

- **`small_house`** = realistic indoor residential. Multi-room, furniture-rich, plenty of features for scan registration. Heavy load on Gazebo (~89 models) so allow 30–60 s to load. Texture paths inside some AWS models are broken (Portrait*.jpg) — these print harmless warnings.
- **`warehouse`** = realistic industrial. Long aisles between shelves create a structured but challenging geometry where LIO-SAM loop closure may not trigger (aisles look similar), but features are rich enough for scan matching. Has a roof (indoor lighting model).
- **`bookstore`** = densest environment (141 AWS models). Retail shelves, tables, narrow aisles. Visually richest — good for showing RKO-LIO robustness under clutter.

**Why not `corridor` / `tunnel` / `warehouse_no_roof` / `basement_room`?** `corridor` and `tunnel` were built for early bring-up with box-wall geometry — not realistic for a paper. `basement_room` and `static_arena` are custom but too sparse. `warehouse_no_roof` has an open ceiling which makes the environment artificially bright and less realistic than the roofed version. We use `warehouse` (with roof) instead.

### Adding a new world

Drop a `.world` SDF in `src/turtlebot_rkolio_sim/worlds/<name>.world`, then:

```bash
cd ~/tb3_3d_ws
colcon build --packages-select turtlebot_rkolio_sim --symlink-install
ros2 launch turtlebot_rkolio_sim sim_with_rkolio.launch.py world:=<name>
```

`--symlink-install` means future world edits don't need a rebuild.

---

## 7. Launch files: who launches what

There are three launchers shipped in the package and three more we wrote for this experiment.

### Shipped in `turtlebot_rkolio_sim`

| Launcher | Spawns | Purpose |
|---|---|---|
| `simulation.launch.py` | Gazebo + robot (stock URDF) | ⚠️ Uses stock TB3 URDF — **TF tree is wrong** for VLP-16/IMU consumers. **Don't use for SLAM**. |
| `sim_with_rkolio.launch.py` | Gazebo + robot + RKO-LIO + RViz | One-shot live RKO-LIO experiment. |
| `sim_with_liosam.launch.py` | Gazebo + robot + ring/time fixer + 4 LIO-SAM nodes + RViz | One-shot live LIO-SAM experiment. |
| `sim_with_fastlio.launch.py` | Gazebo + robot + ring/time fixer + FAST-LIO + RViz | One-shot live FAST-LIO. |
| `teleop.launch.py` | gnome-terminal with teleop_twist_keyboard | Driving. |

### Written for this experiment (`~/simulation_experiment/configs/`)

| Launcher | Spawns | Purpose |
|---|---|---|
| `record_sim_only.launch.py` | Gazebo + robot (**custom URDF**) — no SLAM | Bag recording. Replaces `simulation.launch.py` with correct TF tree. |
| `replay_liosam.launch.py` | robot_state_publisher + ring/time fixer + 4 LIO-SAM nodes + RViz | Bag replay through LIO-SAM. No Gazebo. |
| `replay_rkolio.launch.py` | robot_state_publisher + RKO-LIO + RViz | Bag replay through RKO-LIO. No Gazebo. |

All replay launchers set `use_sim_time:=true` on every node — they expect `/clock` from `ros2 bag play --clock`.

---

## 8. The experiment protocol: bag-replay vs live

There are two ways to compare two SLAM algorithms on a simulated robot:

### A) Live runs
Launch sim + algorithm 1 + driver, drive, record output. Then launch sim + algorithm 2, drive again, record output. Compare.

**Problem:** the two runs see different sensor data — Gazebo physics is stochastic (IMU noise) and human driving is non-repeatable. The comparison conflates algorithmic differences with input differences. Not fair for a paper.

### B) Bag-replay (what we use)
Launch sim only, drive once, record raw sensors into a bag. Stop sim. Then for each algorithm, replay the bag through it offline, record the SLAM output. Compare.

**Benefit:** both algorithms see byte-identical input. Differences in output reflect only algorithmic differences. Standard practice in SLAM papers.

**Caveat:** the replay must use `--clock` so all SLAM nodes (with `use_sim_time:=true`) get bag-driven time. Otherwise IMU integration over wall-clock intervals gives garbage.

> The `~/simulation_experiment/configs/replay_*.launch.py` files are written specifically for protocol B.

### The hybrid we ended up doing

During Phase 6 recording, we also run RKO-LIO live **for visualization only** — so the operator can see a map building in RViz while they drive and notice problems (e.g. driving too fast, robot stuck). The bag still captures only raw sensors, so the LIO-SAM replay later is still fair.

If you want maximum rigor for the paper, **don't** run any SLAM during recording — but expect a blind, boring driving experience.

---

## 9. Why you see "drifts" and "ghost walls"

A common surprise during recording: RKO-LIO's live map shows double walls or smeared rooms. Don't panic — this is **expected behavior** and a useful data point.

### Causes (in rough order of magnitude)

1. **No loop closure in RKO-LIO.** When the robot revisits an area, the new scans land at the algorithm's current best estimate of the pose — which has drifted by some small amount since the first visit. The new scans don't snap onto the old map, so you see "ghost" overlap.
2. **Deskew disabled.** Gazebo doesn't emit per-point timestamps, so RKO-LIO can't undistort scans for in-scan motion. Fast yaw is the worst case — turning at 1 rad/s with a 0.1 s scan period means each scan sweeps through 5.7° of rotation, which smears edges.
3. **IMU rate.** 50 Hz is low for LIO. Both algorithms were designed for 100–500 Hz. The configs compensate with noise tuning, but it's not free.
4. **Driving speed.** Faster = bigger inter-scan motion = bigger registration error per scan.
5. **Texture-poor surfaces.** AWS small_house has plenty of features (furniture). The old `corridor.world` did not — that's part of why it was deprecated.

### What to do about it

- **Drive slow.** Use `q/z` keys in teleop to reduce max linear/angular speeds. ~0.15 m/s and ~0.5 rad/s is comfortable.
- Wide, gentle turns instead of in-place pivots.
- Don't be alarmed when you see drift in RKO-LIO's live map. The RMSE numbers from evo will quantify it; that's the paper's job.
- Expect LIO-SAM to be visually cleaner *when* its loop closure fires, but to fail more often (it's less robust).

### The paper's framing

The narrative is **not** "RKO-LIO is more accurate". It's "RKO-LIO is more **robust** and degrades gracefully where LIO-SAM either succeeds with loop closure or fails catastrophically." The qualitative result you see (drift but no crash in RKO-LIO; clean map or chaos in LIO-SAM) is the story.

---

## 10. Evaluation methodology

We use [`evo`](https://github.com/MichaelGrupp/evo) for trajectory comparison. Three metrics:

### APE — Absolute Pose Error
Direct global comparison between estimated trajectory and ground truth. Reports RMSE / mean / median / std in meters. **Use this for the headline number** ("our drift is X m").

```bash
evo_ape bag2 <gt_bag> <est_bag> <gt_topic> <est_topic> --align --correct_scale
```

`--align` does a SE(3) alignment first (estimated trajectory is up to a rigid transform from GT — both algorithms start with no global anchor). `--correct_scale` adds scale recovery — useful if the estimate has scale drift.

### RPE — Relative Pose Error
Per-step incremental error over a fixed distance window (`--delta 1 --delta_unit m` = error per 1 meter of motion). **Use this to compare drift rate** (RKO-LIO's typical advantage).

### Trajectory overlay
Visual side-by-side of GT, LIO-SAM, RKO-LIO in xy. Best paper figure.

```bash
evo_traj bag2 <gt_bag> <est1_bag> <est2_bag> <gt_topic> <est1_topic> <est2_topic> --ref <gt_topic> --align --plot --plot_mode xy
```

### Ground truth in this experiment

`/odom` from `gazebo_ros_diff_drive` is the ground truth. **Caveat:** wheel odom in Gazebo is also slightly noisy and drifts on collisions. For perfect GT you'd want `gazebo_ros_p3d` (a pose snapshot plugin reading the SDF model pose directly). For this paper's purposes wheel odom is good enough — it doesn't have the LIO-class drift that RKO-LIO/LIO-SAM have, and `--align` removes the constant offset.

If you ever need to switch GT to true model pose, add this to the SDF:

```xml
<plugin name="gazebo_ros_p3d" filename="libgazebo_ros_p3d.so">
  <ros><namespace>/</namespace><remapping>~/out:=/ground_truth/odom</remapping></ros>
  <body_name>base_link</body_name>
  <frame_name>world</frame_name>
  <update_rate>30</update_rate>
</plugin>
```

Then record `/ground_truth/odom` in the bag and use it as the ref topic in evo.

---

## 11. File and directory map

### Experiment artefacts (`~/simulation_experiment/`)

```
simulation_experiment/
├── RUNBOOK.md             # Step-by-step commands. Read this when running.
├── DEVELOPER_GUIDE.md     # This file. Read this to understand WHY.
├── EXPERIMENT_LOG.md      # Append-only journal of what happened each session.
├── configs/
│   ├── record_sim_only.launch.py    # Phase 6 launcher
│   ├── replay_liosam.launch.py      # Phase 7 launcher
│   └── replay_rkolio.launch.py      # Phase 8 launcher
├── bags/                   # ros2 bag2 sqlite dirs
├── results/                # evo zip results + .txt stats + .pdf plots
├── screenshots/            # PNG screenshots for the paper
└── logs/                   # raw stdout/stderr from background launches
```

### Workspace (`~/tb3_3d_ws/`)

```
src/
├── turtlebot_rkolio_sim/
│   ├── urdf/{custom_turtlebot.urdf.xacro, sensors.urdf.xacro}
│   ├── models/custom_turtlebot_rkolio/model.sdf
│   ├── worlds/                          # 16 .world files
│   ├── config/
│   │   ├── liosam_vlp16_sim.yaml        # LIO-SAM params
│   │   ├── sim_rkolio_config.yaml       # RKO-LIO params
│   │   ├── fastlio_vlp16_sim.yaml       # FAST-LIO params
│   │   ├── sim_rviz.rviz                # RViz layout
│   │   └── liosam_rviz.rviz, fastlio_rviz.rviz
│   ├── launch/                          # 5 shipped launchers
│   └── scripts/vlp16_ring_time_fixer.py
├── LIO-SAM/, FAST_LIO/, livox_ros_driver2/, Livox-SDK2/
├── Bonxai/, Sophus/                     # RKO-LIO build deps
└── turtlebot_rkolio_hardware/           # real-robot launch (not used in sim)
```

---

## 12. Reproducing the experiment from a clean checkout

If a colleague clones this repo on a fresh Ubuntu 22.04 / ROS 2 Humble box:

```bash
# 1) Install system deps (Section 2)

# 2) Clone the workspace (or copy from existing user)
mkdir -p ~/tb3_3d_ws/src
cd ~/tb3_3d_ws/src
git clone <repo-with-turtlebot_rkolio_sim>
git clone https://github.com/TixiaoShan/LIO-SAM.git && cd LIO-SAM && git checkout ros2 && cd ..
git clone https://github.com/PRBonn/rko_lio
git clone https://github.com/hku-mars/FAST_LIO
git clone https://github.com/strawlab/python-pcl
# ... + Bonxai, Sophus, Livox-SDK2/livox_ros_driver2

# 3) Build
cd ~/tb3_3d_ws
colcon build --symlink-install

# 4) Source
source install/setup.bash
echo "source ~/tb3_3d_ws/install/setup.bash" >> ~/.bashrc

# 5) Verify
ros2 pkg list | grep -E "turtlebot_rkolio_sim|lio_sam|rko_lio"

# 6) Smoke test
ros2 launch turtlebot_rkolio_sim simulation.launch.py world:=small_house
# (For SLAM use sim_with_rkolio.launch.py, NOT simulation.launch.py — Section 7)

# 7) Run the runbook
xdg-open ~/simulation_experiment/RUNBOOK.md
```

---

## 13. Lessons learned (the painful ones)

### IDs and topics
- The real-robot bridges Pixhawk IMU to `/mavros/imu/data`. The sim follows the same name **on purpose**. If you change it on either side, change it on both.
- LIO-SAM's `pointCloudTopic` is **post-fixer** (`/velodyne_points_proc`). RKO-LIO's `lidar_topic` is **pre-fixer** (`/velodyne_points`). Don't confuse them.
- ROS 2 bag topic names from `ros2 bag info` sometimes display with leading `/` and sometimes without — `evo`'s `bag2` adapter is strict. Always copy-paste from `ros2 bag info`.

### Build / runtime
- The `ros-humble-gtsam` apt package gives GTSAM 4.2. The Borglab PPA gives 4.1. **Mixing them ABI-breaks LIO-SAM.** Pick one and stick with it. We use the apt 4.2.
- `colcon build --symlink-install` is your friend for edits to launchers, configs, URDFs, worlds — no rebuild needed.
- After deleting source for a package, the install/ dir still works **until** you `colcon build` again. Then it tries to rebuild and fails. Be careful around the `rko_lio` package which had its src directory deleted in this workspace — don't run `colcon build --packages-select rko_lio` unless you restore the source.

### Simulation
- AWS RoboMaker model texture paths are broken (relative `../../../../photos/`). The warnings are harmless. The models themselves work.
- Gazebo's first launch for a heavy world (89+ models) can take 60–90 seconds. Be patient on first run.
- `gnome-terminal` for teleop sometimes opens behind the focused window — check your taskbar.

### Driving
- Slow is fast. A 0.15 m/s, 0.5 rad/s drive produces cleaner data than aggressive driving.
- Pause for a second when arriving at a new room — gives the algorithms a frame to stabilize.
- Always close the loop (return near start). LIO-SAM's loop closure is its main feature — give it a chance.

### Recording
- Auto-starting bag recording from a launch file is fragile (rosbag2 doesn't always handle `Ctrl+C` cleanly when launched as a subprocess). Better: open a dedicated terminal for `ros2 bag record`, start/stop manually.
- Always run `ros2 bag info` after recording. If `/velodyne_points` shows < 1800 messages for a 3-minute drive, something dropped — likely Gazebo couldn't keep up.

### evo
- `--align` is almost always what you want. Without it, the absolute frame offset between GT and estimate dominates the RMSE.
- `--correct_scale` is sometimes too generous — it can mask a real problem. For RKO-LIO/LIO-SAM with metric IMU, you don't need it. Leave it off if results look suspicious.
- `evo_res` overwrites the .zip statistics. If you want to keep both world's results, rename the zips.
- **evo 1.36.4 takes ONE bag** — not two. Use merged bags (containing GT `/odom` + SLAM odometry in the same bag). See `~/simulation_experiment/merge_bags.py`.
- Always pass `MPLBACKEND=Agg` when saving plots headlessly (no display). Without it, `evo_res --plot` tries to open a PyQt6 window and crashes if PyQt6 is not installed.
- Use `--use_filenames` with `evo_res` when both zip files have the same internal `est_name` (both named "odometry") — otherwise evo refuses to plot.

### Python / matplotlib — the `mpl_toolkits` namespace package trap

On Ubuntu 22.04 with pip matplotlib 3.x installed alongside system matplotlib 3.5.1, all `evo` plot commands crash with:

```
ImportError: cannot import name 'docstring' from 'matplotlib'
```

**Root cause:** `/usr/lib/python3/dist-packages/matplotlib-3.5.1-nspkg.pth` runs at Python startup and injects the *system* `mpl_toolkits` into `sys.modules` via `setdefault`. Even though user site-packages (`~/.local/lib/python3.10/site-packages`) is first in `sys.path`, this `.pth` file pre-empts the import before any user code runs. The system `mpl_toolkits` was compiled against the old matplotlib API (which had a `docstring` module that was removed in 3.6+).

**Fix:** Create a `usercustomize.py` in user site-packages that runs *after* `site.py` and overrides the poisoned `sys.modules` entry:

```bash
cat > ~/.local/lib/python3.10/site-packages/usercustomize.py << 'EOF'
# Fix mpl_toolkits to use pip-installed version (compatible with matplotlib 3.10+)
# The system matplotlib-3.5.1-nspkg.pth injects the system mpl_toolkits into
# sys.modules at startup; this runs after site.py to override that.
import sys as _sys, os as _os
_user_mpl = '/home/anas/.local/lib/python3.10/site-packages/mpl_toolkits'
if _os.path.isdir(_user_mpl) and 'mpl_toolkits' in _sys.modules:
    _sys.modules['mpl_toolkits'].__path__ = [_user_mpl]
EOF
```

Verify with: `python3 -c "import mpl_toolkits; print(mpl_toolkits.__path__)"` — should show the user path, not `/usr/lib/python3/dist-packages/mpl_toolkits`.

This fix is **persistent** — it survives across terminals and reboots. It was applied on this machine in May 2026 and is already in place.

### Performance
- LIO-SAM at `--rate 1.0` may not keep up on a mid-tier laptop. Use `--rate 0.5` to be safe. RKO-LIO usually handles 1.0 fine.
- RViz with the heavy `liosam_rviz.rviz` (lots of displays) can hit 50% CPU on its own. Close it during evo runs.

---

## 14. Roadmap / what's next

If someone wants to extend this work:

1. **Add `gazebo_ros_p3d` for true ground truth** — see Section 10. Would tighten the APE numbers.
2. **Variable speed driving** — record one bag at 0.1 m/s, another at 0.5 m/s; compare per-algorithm drift sensitivity.
3. **Dynamic obstacles** — drop a couple of moving boxes in the world. Real environments have people walking by; current sim is static.
4. **Add FAST-LIO to the comparison** — the launcher exists, just rerun Phase 7/8/9 with `replay_fastlio.launch.py` (would need to be written, mirroring the LIO-SAM one).
5. **Pre-bagged datasets** — record a "canonical" bag per world and check it into LFS, so others can reproduce numbers without driving themselves.
6. **Automate driving** — write a simple `cmd_vel` publisher that traces a fixed waypoint path. Removes the non-repeatable human driver from the loop and gives reproducible bags.
7. **Real-robot baseline** — RKO-LIO already runs on the real TurtleBot (see `turtlebot_rkolio_hardware/`). A pair of "sim" and "real" runs through the same indoor space would be a strong figure.

---

## 15. Contacts and credits

- **Authors:** Anas Alqadhi (paper lead), Aysegül Uçar (advisor), Bajhaw, Munef.
- **Paper:** *Drift-Resilient Indoor 3D Mapping* — ICHORA 2026, Paper ID 331.
- **Algorithms:**
  - LIO-SAM: Shan, Englot, Meyers, Wang, Ratti, Rus, *IROS 2020*. github.com/TixiaoShan/LIO-SAM
  - RKO-LIO: PRBonn, *2024*. github.com/PRBonn/rko_lio
  - FAST-LIO: Xu et al., *2021*. github.com/hku-mars/FAST_LIO
- **Worlds:** AWS RoboMaker (Apache 2.0) — github.com/aws-robotics
- **TurtleBot3:** ROBOTIS — robotis.com/turtlebot3

---

End of developer guide sections 1–15. Sections 16–23 below document the implementation internals, session history, and everything else that was learned in practice.

---

## 16. `use_sim_time` and the `/clock` contract

This is the single most confusing thing for anyone new to bag-replay with ROS 2. Get it wrong and your SLAM maps will be garbage or the nodes will stall entirely.

### How ROS 2 time works

Every ROS 2 node has a clock source. When `use_sim_time: true`, the node's `now()` returns whatever the last `/clock` message said. When `use_sim_time: false` (the default), it returns wall-clock time.

### During Gazebo simulation (recording phase)

Gazebo publishes `/clock` in real time. All our nodes set `use_sim_time: true`, so they follow Gazebo's clock. This is fine and transparent.

### During bag replay (evaluation phase)

`ros2 bag play <bag> --clock` makes the bag player act as a `/clock` publisher, advancing time at whatever `--rate` you specify. All SLAM nodes set `use_sim_time: true` — so they follow the bag's clock.

**Critical:** if you forget `--clock` and run `ros2 bag play <bag>` without it, the bag doesn't publish `/clock`. The SLAM nodes are waiting for `/clock` to advance but it never does. They appear frozen. The bag data plays (topics arrive) but all node timestamps are stuck at 0. IMU integration becomes meaningless. Maps break completely.

**Also critical:** `ros2 bag play` without `--clock` still publishes topics with the original bag timestamps in the message headers. But the SLAM node's internal clock (`node->now()`) stays at 0. When LIO-SAM's `imageProjection` compares the scan stamp against `node->now()` to decide whether a scan is "current", it sees a huge discrepancy and discards every scan.

### The `static_transform_publisher` subtlety

Both `replay_liosam.launch.py` and `replay_rkolio.launch.py` include:

```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    ...
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    parameters=[{'use_sim_time': True}],
)
```

This publishes a zero `map → odom` static TF. Without it, RViz can't resolve the `map` frame and shows nothing. LIO-SAM's `mapOptimization` node expects to publish into the `map` frame — this node provides the anchor.

In a real deployment with loop closure active, the `map → odom` TF will be updated by LIO-SAM dynamically as the loop fires. The static publisher is just to get something in there at startup.

### Checking if time is flowing correctly

```bash
# While bag is playing and SLAM running:
ros2 topic echo /clock --once                # should show a non-zero sim time
ros2 topic echo /tf --once | grep stamp      # header.stamp should track /clock
```

If `/clock` shows `sec: 0, nanosec: 0`, the bag was played without `--clock`.

---

## 17. Full annotated config files

These are the deployed configs as of May 2026. They live in
`~/tb3_3d_ws/install/turtlebot_rkolio_sim/share/turtlebot_rkolio_sim/config/`.

### 17.1 `liosam_vlp16_sim.yaml` — annotated

```yaml
/**:
  ros__parameters:

    # ── Topics ──────────────────────────────────────────────────────────────
    # Post-fixer topic (with ring + time fields injected by vlp16_ring_time_fixer.py)
    pointCloudTopic: "/velodyne_points_proc"
    # Pixhawk via MAVROS — same name as real robot for config reuse
    imuTopic: "/mavros/imu/data"
    odomTopic: "odometry/imu"    # internal topic, LIO-SAM publishes here
    gpsTopic: "odometry/gpsz"    # GPS disabled — name is irrelevant

    # ── Frames ──────────────────────────────────────────────────────────────
    lidarFrame: "velodyne"
    baselinkFrame: "base_link"
    odometryFrame: "odom"
    mapFrame: "map"

    # ── GPS / heading init ──────────────────────────────────────────────────
    useImuHeadingInitialization: false   # no magnetometer or GPS
    useGpsElevation: false

    # ── Sensor model ────────────────────────────────────────────────────────
    sensor: velodyne          # selects VLP-16 ring projection model
    N_SCAN: 16                # number of vertical beams
    Horizon_SCAN: 1800        # horizontal resolution — MUST match Gazebo plugin samples
    downsampleRate: 1         # no downsampling
    lidarMinRange: 0.3        # drop points closer than 0.3 m (self-returns)
    lidarMaxRange: 20.0       # indoor cap; real VLP-16 goes to 100 m

    # ── IMU noise (CRITICAL — tuned for 50 Hz Pixhawk Cube Orange+) ─────────
    # LIO-SAM was designed for 100–400 Hz high-quality IMUs.
    # At 50 Hz the sampled gyro/accel noise is higher than at 200 Hz
    # because high-frequency vibration isn't averaged away.
    # We raise these values so the factor graph trusts scan-matching more
    # than IMU integration. If you upgrade to 200 Hz, lower these by ~2×.
    imuAccNoise: 0.02       # m/s^2 — was 3.99e-3 in default; raised 5× for 50 Hz
    imuGyrNoise: 0.005      # rad/s — was 1.56e-3 in default; raised 3× for 50 Hz
    imuAccBiasN: 0.001      # accelerometer bias random walk
    imuGyrBiasN: 0.0001     # gyroscope bias random walk
    imuGravity: 9.80511     # Gazebo default gravity
    imuRPYWeight: 0.01      # how much IMU roll/pitch corrects LiDAR odometry

    # ── Extrinsics: Pixhawk → VLP-16 (IMU frame to LiDAR frame) ─────────────
    # pixhawk_cube_link in URDF is at (0.06, 0.0, 0.272) from base_link.
    # velodyne link is at (0.0, 0.0, 0.325) from base_link.
    # Translation IMU→LiDAR = LiDAR_pos - IMU_pos = (-0.06, 0.0, +0.053).
    # Both are aligned with base_link axes, so rotation is identity.
    extrinsicTrans: [-0.06, 0.0, 0.053]
    extrinsicRot:   [1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0]
    extrinsicRPY:   [1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0]

    # ── LOAM feature extraction ─────────────────────────────────────────────
    edgeThreshold: 1.0          # curvature threshold for edge (corner) points
    surfThreshold: 0.1          # curvature threshold for surface (planar) points
    edgeFeatureMinValidNum: 10  # min edges per scan to proceed
    surfFeatureMinValidNum: 100 # min surfaces per scan to proceed

    # ── Voxel filters ───────────────────────────────────────────────────────
    odometrySurfLeafSize: 0.2      # 20 cm voxel for odometry surface features
    mappingCornerLeafSize: 0.1     # 10 cm for map corner features
    mappingSurfLeafSize: 0.2       # 20 cm for map surface features

    # ── Motion constraints (TurtleBot3 is a flat ground robot) ─────────────
    z_tollerance: 0.3          # drop keyframes with >0.3 m z-translation
    rotation_tollerance: 1000.0  # effectively disabled — 1000 rad/s is impossible

    # ── CPU ─────────────────────────────────────────────────────────────────
    numberOfCores: 4
    mappingProcessInterval: 0.15   # seconds between map optimizations

    # ── Keyframe spacing ────────────────────────────────────────────────────
    surroundingkeyframeAddingDistThreshold: 0.5    # new keyframe every 0.5 m
    surroundingkeyframeAddingAngleThreshold: 0.2   # or every 11° yaw

    # ── Loop closure ────────────────────────────────────────────────────────
    loopClosureEnableFlag: true           # THE headline feature
    loopClosureFrequency: 1.0             # Hz — check for loops once per second
    surroundingKeyframeSize: 50           # use 50 nearest keyframes for local map
    historyKeyframeSearchRadius: 10.0     # only look for loops within 10 m
    historyKeyframeSearchTimeDiff: 30.0   # only match keyframes > 30 s apart
    historyKeyframeSearchNum: 25          # number of history keyframes to check
    historyKeyframeFitnessScore: 0.3      # ICP score threshold for accepting a loop
```

### 17.2 `sim_rkolio_config.yaml` — annotated

```yaml
/**:
  ros__parameters:
    # ── Topics / frames ────────────────────────────────────────────────────
    lidar_topic: "/velodyne_points"    # raw — no ring/time fixer needed
    imu_topic:   "/mavros/imu/data"    # same as LIO-SAM for config reuse
    base_frame:  "base_link"
    imu_frame:   "imu_link"
    lidar_frame: "velodyne"
    odom_frame:  "odom"
    odom_topic:  "/rko_lio/odometry"   # published output

    # ── Key settings ────────────────────────────────────────────────────────
    # deskew=false because Gazebo's plugin emits no per-point timestamps.
    # On the real robot with the Velodyne driver, set this to true.
    deskew:               false

    # initialization_phase=true runs a static initialization phase where
    # the robot must stay still for a few seconds so RKO-LIO can estimate
    # initial IMU bias. On the real robot this was found to cause an
    # SO3::exp crash (commit 382dce4) so it is set to false in hardware config.
    initialization_phase: true

    max_iterations:       100
    convergence_criterion: 0.00001
    max_num_threads:      0       # 0 = use all available cores

    # ── Map voxel size ──────────────────────────────────────────────────────
    # 0.3 m is good for indoor environments. Smaller (0.1–0.2) = denser map
    # but much more RAM and slower. Larger (0.5) = faster but misses details.
    voxel_size:           0.3
    double_downsample:    true
    max_points_per_voxel: 20

    # ── Range limits ────────────────────────────────────────────────────────
    max_range:  20.0    # indoor cap — prevents far-range noise from open ceilings
    min_range:   0.9    # drops chassis self-returns; VLP-16 has ~0.5 m blind spot

    max_correspondance_distance: 0.5

    # ── IMU dynamics (tuned for 50 Hz Pixhawk) ─────────────────────────────
    # min_beta is the damping factor for the kinematic model.
    # Higher = trust IMU prediction more, LiDAR correction less.
    # 150.0 was tuned empirically for this hardware.
    min_beta:          150.0
    max_expected_jerk: 5.0     # m/s^3; outlier rejection for acceleration spikes
    imu_buffer_size:   4000    # how many IMU messages to buffer (80 s at 50 Hz)
    lidar_buffer_size: 50      # how many LiDAR scans to buffer

    # ── Output ──────────────────────────────────────────────────────────────
    publish_local_map:     true    # /rko_lio/local_map for RViz
    map_topic:             "/rko_lio/local_map"
    publish_deskewed_scan: true    # publishes the corrected scan (even if deskew=false)
    invert_odom_tf:        false
```

---

## 18. Full annotated launch files

### 18.1 `record_sim_only.launch.py`

```python
# Key design decisions:

# 1. Uses custom URDF (not stock TB3)
urdf_file = os.path.join(sim_pkg, 'urdf', 'custom_turtlebot.urdf.xacro')
# The stock simulation.launch.py uses the TB3 URDF which puts velodyne and
# imu_link at the wrong positions. This launcher fixes that.

# 2. Spawns via SDF (physics fidelity) but publishes TF via URDF
# SDF = Gazebo model with correct sensor plugins, noise, physics
# URDF = robot_description for robot_state_publisher (TF tree for ROS consumers)
sdf_file = os.path.join(sim_pkg, 'models', 'custom_turtlebot_rkolio', 'model.sdf')

# 3. GAZEBO_MODEL_PATH includes turtlebot3_gazebo models
# The AWS worlds reference models from TB3's model library.
# Without this, small_house loads with ~89 missing models (invisible geometry).
tb3_gazebo_models = os.path.join(
    get_package_share_directory('turtlebot3_gazebo'), 'models')

# 4. world argument (no .world extension)
# The .world extension is added in the substitution:
world_file = PathJoinSubstitution([sim_pkg, 'worlds', LaunchConfiguration('world')])
# → world:=[sim_pkg]/worlds/small_house.world
```

### 18.2 `replay_liosam.launch.py`

```python
# Key design decisions:

# 1. TimerAction delays
delayed_slam = TimerAction(period=2.0, actions=[fixer_node] + lio_nodes)
delayed_rviz = TimerAction(period=1.0, actions=[rviz])
# robot_state_publisher starts first (it must be up before SLAM nodes request TF).
# RViz starts after 1 s — gives it time to find robot_description before drawing.
# SLAM nodes start after 2 s — gives robot_state_publisher and RViz time to settle.

# 2. static_tf_map_odom
static_tf_map_odom = Node(
    package='tf2_ros', executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
)
# LIO-SAM's mapOptimization publishes in the 'map' frame. Without a map→odom
# transform, RViz can't visualize the output and tf2 lookups fail at startup.
# LIO-SAM will overwrite this with a dynamic TF once it runs — this is just
# the initial anchor.

# 3. LIO-SAM consumes /velodyne_points_proc, not /velodyne_points
# The fixer_node subscribes to /velodyne_points and publishes /velodyne_points_proc.
# The liosam_vlp16_sim.yaml sets pointCloudTopic: "/velodyne_points_proc".
# This chain must be intact or LIO-SAM will silently discard every scan.

# 4. All nodes get use_sim_time: True
# The bag is played with --clock, so /clock drives all node clocks.
common_params = [params_file, {'use_sim_time': True}]
```

### 18.3 `replay_rkolio.launch.py`

```python
# Key design decisions:

# 1. No ring/time fixer
# RKO-LIO reads /velodyne_points directly. No fixer node in this launcher.

# 2. Topic remapping
remappings=[
    ('pointcloud', '/velodyne_points'),
    ('imu', '/mavros/imu/data'),
],
# RKO-LIO's online_node listens on 'pointcloud' and 'imu' by default
# (relative names). These remappings make them absolute.

# 3. Same static_tf_map_odom trick
# RKO-LIO publishes odometry in the odom frame (not map frame), but
# RViz needs the chain odom→map to resolve fixed frame='map'.
```

---

## 19. `vlp16_ring_time_fixer.py` — implementation walkthrough

This is one of the most non-obvious components in the whole system. Read this before modifying it.

### Why it exists

The Gazebo `libgazebo_ros_ray_sensor` plugin generates a VLP-16 point cloud with only four fields: `x`, `y`, `z`, `intensity`. Real Velodyne hardware drivers (and the `velodyne_driver` ROS package) emit two additional fields:

- `ring` (uint16): which of the 16 laser beams produced this point (0 = bottom beam at −15°, 15 = top beam at +15°)
- `time` (float32): relative timestamp within the scan in seconds (0 = start of scan, 0.1 = end of scan)

LIO-SAM's `imageProjection` node uses `ring` to organize the unordered cloud back into a 16×1800 range image. Without `ring`, it fails to project and discards every scan. It uses `time` to deskew the scan (correct for robot motion during the 0.1 s scan acquisition window). Without `time`, deskewing is skipped (or worse, done with zeroed timestamps).

### The reconstruction algorithm

**Ring assignment** (elevation → beam index):
```python
VLP16_ANGLES_RAD = np.deg2rad(np.arange(-15, 16, 2))  # [-15, -13, ..., +13, +15]
r_xy = np.sqrt(xs*xs + ys*ys)
elev = np.arctan2(zs, r_xy)                            # elevation of each point
rings = np.argmin(np.abs(elev[:, None] - VLP16_ANGLES_RAD[None, :]), axis=1)
```
For each point, compute its elevation angle, then find the nearest VLP-16 beam by minimizing `|elevation − beam_angle|`. This is O(n × 16) but n ≈ 14,000 points per scan, so it's fast with numpy broadcasting.

**Time assignment** (azimuth → fractional scan time):
```python
az = np.arctan2(-ys, xs)             # azimuth [-π, π]
az = np.where(az < 0, az + 2*np.pi, az)  # normalize to [0, 2π]
times = az / (2 * np.pi) * SCAN_PERIOD  # SCAN_PERIOD = 0.1 s
```
The VLP-16 spins counter-clockwise (when viewed from above). Each scan starts at azimuth 0° and completes at 360° in 0.1 s. A point at azimuth θ was acquired at time `θ / 360° × 0.1 s` into the scan. The `arctan2(-y, x)` convention matches the Velodyne rotation direction.

### Output format

The output cloud has `point_step = 24` bytes per point:
```
offset  0: x         (float32, 4 bytes)
offset  4: y         (float32, 4 bytes)
offset  8: z         (float32, 4 bytes)
offset 12: intensity (float32, 4 bytes)
offset 16: ring      (uint16,  2 bytes)
offset 18: [padding] (2 bytes)
offset 20: time      (float32, 4 bytes)
```
This matches the wire format expected by LIO-SAM's parser.

### Accuracy limitations

- The `ring` field is synthetic — Gazebo samples all 16 beams in a single ray trace step, so points are not actually emitted beam-by-beam in time order as a real VLP-16 does. The elevation-to-ring mapping is geometrically correct but the temporal ordering within a ring is not.
- The `time` field is an approximation. Real per-point timestamps are accurate to microseconds; these are computed from azimuth using the nominal 10 Hz scan rate. If Gazebo drops frames (under load) the 0.1 s assumption breaks.
- Both approximations are good enough for LIO-SAM scan registration in simulation. They would not be appropriate for submitting to a real-robot benchmark.

### Performance

At 10 Hz × 14,400 points/scan, the fixer must process ~144,000 points/second. The numpy vectorised implementation typically runs in 2–5 ms per scan on a modern laptop — well within the 100 ms budget. Watch for slowdowns on older hardware.

---

## 20. Simulation vs real robot: key differences

This section is for anyone who ran the sim experiment and now wants to deploy on the real TurtleBot3 + VLP-16 + Pixhawk.

| Aspect | Simulation | Real Robot |
|---|---|---|
| Clock source | Gazebo `/clock`, driven by simulation | Wall clock |
| `use_sim_time` | `true` everywhere | `false` everywhere |
| `ros2 bag play` flag | `--clock` required | N/A |
| LiDAR topic | `/velodyne_points` (raw x/y/z/intensity only) | `/velodyne_points` (with `ring` + `time` from driver) |
| Ring/time fixer | Required for LIO-SAM | **Not needed** — driver provides fields natively |
| IMU topic | `/mavros/imu/data` at 50 Hz | `/mavros/imu/data` at 50 Hz initially, then 200 Hz after service call |
| IMU frequency | 50 Hz (no upgrade in sim) | Upgradeable to 200 Hz via MAVLink service |
| LIO-SAM `imuFrequence` | 50 | 200 (after upgrade) |
| LIO-SAM noise params | Raised (see config) | Can be tightened at 200 Hz |
| RKO-LIO `deskew` | `false` | `true` (driver provides per-point time) |
| RKO-LIO `initialization_phase` | `true` | **`false`** — causes SO3::exp crash on hardware |
| Ground truth | `/odom` from `gazebo_ros_diff_drive` | None (use evo with external GT if available) |
| Velodyne IP | Simulated, no network config needed | `192.168.8.201` (LiDAR-only network) or `10.37.17.xxx` (hotspot) |
| ROS_DOMAIN_ID | `30` (sim) | `45` (real robot) |
| Start command | launch files in `~/simulation_experiment/configs/` | `bash ~/tb3_3d_ws/start_robot_rkolio.sh` |

### The `initialization_phase` real-robot bug

When `initialization_phase: true`, RKO-LIO runs a startup phase where it computes:
```
R = SO3::exp(ω × t)
```
where `ω` is the average angular velocity over the first few IMU samples. In simulation, these samples are clean synthetic values and the computation is well-conditioned. On the real Pixhawk, the first few IMU samples can have very large values (the Pixhawk takes a moment to stabilize its EKF output). If `|ω × t|` is large, `SO3::exp` overflows or hits a degenerate quaternion, crashing the node.

**Fix:** set `initialization_phase: false` in the hardware config and hold the robot still for 3–5 seconds after launching before moving. The IMU bias will still be estimated; it just uses a different (more robust) initialization path.

This was discovered in commit `382dce4` of `turtlebot_rkolio_hardware`.

### Sourcing the workspace

Sim workflow:
```bash
source /opt/ros/humble/setup.bash
source ~/tb3_3d_ws/install/setup.bash
export ROS_DOMAIN_ID=30
```

Real robot (on Jetson):
```bash
source /opt/ros/humble/setup.bash
source ~/tb3_3d_ws/install/setup.bash
export ROS_DOMAIN_ID=45
export TURTLEBOT3_MODEL=burger
```

Both are already in the respective `~/.bashrc` files.

---

## 21. Disk space management

Bags are large. A 4-minute drive at full sensor rate in `small_house` produces approximately:

| Topic | Rate | Bytes/msg | 4-min total |
|---|---|---|---|
| `/velodyne_points` | 10 Hz | ~230 KB | ~552 MB |
| `/mavros/imu/data` | 50 Hz | ~200 B | ~2.4 MB |
| `/odom` | 30 Hz | ~100 B | ~0.72 MB |
| `/tf` | ~30 Hz | ~500 B | ~3.6 MB |
| `/tf_static` | 1 msg total | ~500 B | negligible |
| `/clock` | ~100 Hz | ~16 B | ~0.38 MB |
| **Total** | | | **~560 MB** |

For the full experiment (raw bag + 2 SLAM output bags):
- `sim_small_house_01`: ~560 MB (raw)
- `liosam_output_small_house_01`: ~50 MB (odometry + registered cloud)
- `rkolio_output_small_house_01`: ~50 MB (odometry + local map)
- evo results + PDFs: ~10 MB
- **Grand total per world: ~670 MB**

For two worlds (small_house + warehouse): ~1.4 GB minimum.

### Before you start recording

```bash
# Check available space
df -h ~/

# Quick cleanup — remove old build logs (safe to delete)
rm -rf ~/tb3_3d_ws/log/build_*

# If still tight — the build/ directory can be deleted and rebuilt
# (DON'T do this unless you have >30 min for a rebuild)
# du -sh ~/tb3_3d_ws/build/
```

As of the start of this experiment the disk was at 90% used (13 GB free of 129 GB). After log cleanup it freed ~2 GB. Monitor during recording:

```bash
# In a separate terminal while recording:
watch -n 5 df -h ~/
```

### If you run out of space mid-recording

The bag recorder will stop writing and print an error. The bag file will be truncated but still parseable up to the last complete message. Run `ros2 bag info` to check what was captured. If you got at least 3 minutes, it may still be usable.

To free space immediately without deleting results:
```bash
# Delete the failed/partial bag
rm -rf ~/simulation_experiment/bags/<partial_bag_name>

# Free pip cache
pip3 cache purge

# Free apt cache
sudo apt clean
```

---

## 22. Session history — what actually happened

This section is the narrative version of `EXPERIMENT_LOG.md`. It exists so a future developer understands the decisions made, not just the commands run.

### Session 1 — May 20, 2026

**Starting state:** The `turtlebot_rkolio_sim` package existed and was fully built with working launchers (`sim_with_rkolio.launch.py`, etc.). LIO-SAM was built. `rko_lio` install was intact but source had been deleted. The workspace was already configured — all the heavy lifting from the master instruction file (Phases 1–5) had been done in prior sessions.

**Disk situation:** 90% full (13 GB free on a 129 GB partition). This dominated early decisions — we couldn't afford a large test bag.

**What happened:**
1. Ran system check (Step 0). Found workspace is `~/tb3_3d_ws`, not `~/ros2_ws` as the master plan assumed. Adjusted all paths.
2. Installed `evo` via pip. Verified `evo_ape --help` works.
3. Chose worlds: `small_house` (primary) + `warehouse_no_roof` (secondary). Rejected `corridor`, `tunnel`, `basement_room` — too artificial for paper results.
4. Created `~/simulation_experiment/` directory structure.
5. Wrote three standalone launch files:
   - `record_sim_only.launch.py` — Gazebo only, correct custom URDF (fixes TF bug in stock `simulation.launch.py`)
   - `replay_liosam.launch.py` — SLAM-only for bag replay, no Gazebo
   - `replay_rkolio.launch.py` — SLAM-only for bag replay, no Gazebo
6. Smoke-tested `record_sim_only.launch.py world:=small_house`. All topics verified: `/velodyne_points` (14,374 pts/scan), `/mavros/imu/data` (50 Hz), `/odom`, `/tf`, `/clock`. TF tree spot-check passed.
7. Attempted Phase 6 (recording) with auto-start from inside the Claude Code session. This was **confusing** — Anas couldn't tell when recording was active, the RKO-LIO live map showed drift and he thought it was a bug (it isn't), and the auto-started bag recorder was hard to stop cleanly.
8. Killed everything, deleted the partial bag (~600 MB, about 2–3 minutes recorded but not confirmed good), and switched strategy.
9. Wrote `RUNBOOK.md` — a step-by-step guide where **Anas controls everything** (when to start/stop recording, when to drive). This replaced the auto-pilot approach.

**Key insight from this session:** Auto-driving or auto-starting recording from an AI assistant is fragile for this kind of experiment. The human needs direct control of the bag recorder terminal. The RUNBOOK ensures this.

**State at end of session:** No bag recorded yet. All infrastructure (launch files, RUNBOOK, evo) ready. Anas ready to run Phase 6 himself.

### Session 1 continued — Phase 6–9 execution (May 20, 2026)

**Phase 6 — bag recording (`sim_small_house_04`):**
- Several aborted attempts (`_01` through `_03`) due to startup sequencing issues (recorder started before SLAM initialized) and TF flickering (bag `/tf` conflicted with `robot_state_publisher`).
- **Fix:** always exclude `/tf` and `/tf_static` from the source bag when replaying: `--topics /velodyne_points /mavros/imu/data /clock`. These topics conflict with the running `robot_state_publisher` during replay.
- Final good bag: `sim_small_house_04` — 349 s, 901 velodyne scans (at 2.58 Hz — Gazebo was throttled to ~27% real-time due to the heavy small_house world), 4501 IMU messages.
- Note: The velodyne rate is **2.58 Hz**, not 10 Hz — Gazebo couldn't keep up. This is expected for an 89-model world on a mid-tier laptop. The SLAM replay at `--rate 0.5` compensates.

**Phase 7 — LIO-SAM replay:**
- Output bag: `liosam_output_small_house_04` — 185 odometry messages over 82 s (after sync-trimming by evo).
- RViz Decay Time on "LIO-SAM Registered Cloud" was set to 600 s (accumulates all keyframe clouds) for the map screenshot.

**Phase 8 — RKO-LIO replay:**
- Output bag: `rkolio_output_small_house_04` — 484 odometry messages over 80 s.

**Phase 9 — evo evaluation:**

evo 1.36.4 only accepts ONE bag. Both output bags were merged with GT using:
```bash
python3 ~/simulation_experiment/merge_bags.py \
  ~/simulation_experiment/bags/sim_small_house_04/ \
  ~/simulation_experiment/bags/liosam_output_small_house_04/ \
  ~/simulation_experiment/bags/liosam_merged_04/
```
See `~/simulation_experiment/merge_bags.py` for implementation.

Plots crashed with `ImportError: cannot import name 'docstring' from 'matplotlib'` — the mpl_toolkits namespace package trap (see Section 13). Fixed with `usercustomize.py`.

**Confirmed results — small_house_04:**

| Metric | LIO-SAM | RKO-LIO |
|--------|--------:|--------:|
| APE RMSE (m) | 12.075 | **0.076** |
| APE Mean (m) | 5.958 | **0.071** |
| APE Median (m) | 1.213 | **0.067** |
| APE Max (m) | 40.528 | **0.129** |
| RPE RMSE (m/m) | 25.233 | **0.030** |
| RPE Mean (m/m) | 12.932 | **0.026** |

**LIO-SAM divergence analysis:**
- t = 0–60 s: LIO-SAM tracks well. APE is stable at ~1.2 m (systematic offset, not drift — this is the initial alignment residual after Umeyama).
- t = 67.3 s: APE first crosses 5 m — abrupt jump, not gradual drift.
- t = 72.8 s: APE > 10 m.
- t = 74.7 s: APE > 20 m.
- t = 82 s (end): APE 37–40 m.
- **Root cause:** a failed or wrong loop closure fired at ~t=67 s and corrupted the GTSAM factor graph pose estimate. The jump is instantaneous (characteristic of a bad loop closure insertion, not incremental scan-matching drift). LIO-SAM's `historyKeyframeFitnessScore: 0.3` threshold may have been too loose for the 2.58 Hz throttled scans.
- **RKO-LIO comparison:** APE is flat and tight throughout (0.016–0.129 m). No catastrophic failure. This is the paper's core argument.

**IEEE figures generated:**
- `trajectory_overlay_ieee.pdf/.png` — combined overlay, single-column (3.5"), Times New Roman serif, 300 DPI, fonts embedded (PDF type 42)
- `liosam_ape_small_house.pdf`, `rkolio_ape_small_house.pdf` — individual APE plots
- `comparison_ape_small_house.pdf` — side-by-side bar chart
- `liosam_trajectory.pdf`, `rkolio_trajectory.pdf` — individual trajectory overlays
- `liosam_rpe_small_house.pdf`, `rkolio_rpe_small_house.pdf` — RPE plots

Script: `~/simulation_experiment/plot_trajectory_overlay.py`

**State at end of session:** small_house experiment fully complete. All results confirmed. Next: repeat Phases 6–9 for `warehouse` and `bookstore` worlds.

### Session 2 — May 20, 2026 (warehouse + bookstore + full evaluation)

**Starting state:** `sim_small_house_06` bag recorded at 0.15 m/s. All three environments needed to be done and evaluated.

**Work completed in this session:**

1. **RViz decay time fix:** Set `Decay Time = 0` on all point cloud displays in `sim_rviz.rviz`. This eliminated the flickering/accumulation artifact where old point clouds lingered in the display.

2. **Teleop speed cap:** Hard-capped `teleop.launch.py` at `speed:=0.15 turn:=0.5` (matches real TurtleBot3 operating conditions at the ICHORA demo). No manual speed adjustment needed by the operator.

3. **Auto-stop replay scripts written:**
   - `~/simulation_experiment/replay_liosam.sh <source_bag_dir> <output_bag_name>` — starts recorder, runs bag replay, stops recorder cleanly
   - `~/simulation_experiment/replay_rkolio.sh <source_bag_dir> <output_bag_name>` — same for RKO-LIO
   - `~/simulation_experiment/run_all_replays.sh` — batch script runs all 4 SLAM replays in series, restarting SLAM between each environment (prevents map contamination)

4. **GAZEBO_RESOURCE_PATH fix for bookstore:** The bookstore AWS world uses `file://models/...` URIs for mesh paths instead of the standard `model://...`. This caused all bookstore models to load as invisible geometry (Gazebo reported them as loaded but rendered nothing). Fix: set `GAZEBO_RESOURCE_PATH=~/.gazebo` in `record_sim_only.launch.py`. Warehouse and small_house use `model://` and are unaffected.

5. **Bags recorded:**
   - `sim_small_house_06` — 833 s, 0.15 m/s cap
   - `sim_warehouse_06` — 626 s, 0.15 m/s cap
   - `sim_bookstore_01` — 1470 s, 0.15 m/s cap (bookstore is denser, slower drive)

6. **All 4 SLAM replays completed:**
   - `liosam_output_warehouse_06` — 1249 s wall time, 1354 odometry messages
   - `rkolio_output_warehouse_06` — 1251 s wall time
   - `liosam_output_bookstore_01` — 2933 s wall time, 783 odometry messages (diverged early)
   - `rkolio_output_bookstore_01` — 2938 s wall time, 2648 odometry messages

7. **All 4 bag merges completed** using `merge_bags.py`.

8. **Full evo evaluation (14 commands)** — APE, RPE, trajectory, and comparison for all 3 environments.

9. **All PDF figures generated** — APE, RPE, trajectory overlay, and comparison for all 3 environments.

**Key discovery — LIO-SAM bookstore failure:** LIO-SAM diverged completely in the bookstore environment despite running at proper 0.15 m/s speed. Estimated path length: **2632 m** vs ground truth **43 m** (60× error). This is a genuine algorithmic failure, not a speed or configuration issue. Root cause: the bookstore's dense, repetitive shelving causes scan-matching ambiguity that LIO-SAM's ICP + IMU preintegration cannot recover from. RKO-LIO remained stable throughout (APE RMSE 0.494 m).

**Important note on batch replay script bug:** The initial `run_all_replays.sh` started LIO-SAM once and replayed warehouse + bookstore bags sequentially without restarting SLAM. This contaminated the bookstore map with warehouse keyframes. Fixed by restarting SLAM between each environment — each run is now `start_slam → replay → stop_slam`.

**State at end of session:** All data collected. All figures generated. Full results table ready for the paper. No further experimental work needed.

### Why the original worlds were rejected

Early in the bring-up (before Session 1), the `corridor`, `tunnel`, and `basement_room` worlds were built specifically for this project. They are simple SDF files with box-wall geometry. When tested:

- Visually: "the walls are too high and it looks like a box" — not realistic for an indoor mapping paper
- SLAM quality: sparse features (flat walls only) made registration harder than necessary
- Paper optics: a trajectory overlay plot in a featureless box is not convincing

AWS RoboMaker worlds (`small_house`, `warehouse_no_roof`) have furniture, shelves, and realistic interior features. They load heavier but produce meaningful results.

### The `simulation.launch.py` TF bug — origin story

The stock `turtlebot_rkolio_sim/launch/simulation.launch.py` was the original launcher. It uses the standard TurtleBot3 URDF from `turtlebot3_description` (the system package). That URDF defines `velodyne` and `imu_link` at stock positions that don't match our custom Jetson+Pixhawk stack.

When LIO-SAM ran with this TF tree, the extrinsic calibration was implicitly wrong — the `velodyne → base_link` transform didn't match the `extrinsicTrans` value in the config. The SLAM maps weren't visibly wrong (both frames were still close), but the IMU-LiDAR fusion had a systematic error.

The fix was to write `record_sim_only.launch.py` using `custom_turtlebot.urdf.xacro`, which is the URDF that was specifically modelled to match the SDF positions of the Jetson housing + Pixhawk cube + VLP-16.

**Lesson:** always verify `ros2 run tf2_tools view_frames` and check that the TF tree matches what your SLAM config expects, especially the `velodyne → base_link` and `imu_link → base_link` transforms.

### The GTSAM version saga

The master instruction file said to add the Borglab PPA for GTSAM 4.1. Don't do this. The `ros-humble-lio-sam` package (and the workspace build of LIO-SAM) links against `ros-humble-gtsam 4.2.0` from the standard ROS apt repository. Mixing PPA 4.1 with apt 4.2 causes an ABI mismatch that manifests as a cryptic crash in `mapOptimization` at runtime (not at build time). The workspace was already on apt 4.2 and it works fine. Never install from the Borglab PPA on this system.

---

## 23. Paper and presentation deliverables map

The ICHORA 2026 paper (Paper ID 331) and its associated presentation (`ICHORA2026_RKO_LIO_Presentation.pptx`) need specific files from this experiment. This section maps every placeholder to its source.

### Paper: Section IV-C (Simulation Results) — figures needed

| Figure | Description | Source file | Status |
|---|---|---|---|
| Fig. 5a | Trajectory overlay — small_house world | `~/simulation_experiment/results/liosam_trajectory_small_house.pdf` (LIO-SAM) + `rkolio_trajectory_small_house.pdf` (RKO-LIO) | ✅ generated |
| Fig. 5b | Trajectory overlay — warehouse world | `~/simulation_experiment/results/liosam_trajectory_warehouse.pdf` + `rkolio_trajectory_warehouse.pdf` | ✅ generated |
| Fig. 5c | Trajectory overlay — bookstore world | `~/simulation_experiment/results/liosam_trajectory_bookstore.pdf` + `rkolio_trajectory_bookstore.pdf` | ✅ generated |
| Fig. 6a | APE comparison — small_house | `~/simulation_experiment/results/comparison_ape_small_house.pdf` | ✅ generated |
| Fig. 6b | APE comparison — warehouse | `~/simulation_experiment/results/comparison_ape_warehouse.pdf` | ✅ generated |
| Fig. 6c | APE comparison — bookstore | `~/simulation_experiment/results/comparison_ape_bookstore.pdf` | ✅ generated |

### Paper: Table III (Simulation APE/RPE) — numbers needed

Run this after completing evo evaluation to extract all numbers:

```bash
echo "=== APE LIO-SAM ===" && grep -E "rmse|mean|median|std" ~/simulation_experiment/results/liosam_ape_small_house.txt
echo "=== APE RKO-LIO ===" && grep -E "rmse|mean|median|std" ~/simulation_experiment/results/rkolio_ape_small_house.txt
echo "=== RPE LIO-SAM ===" && grep -E "rmse|mean|median|std" ~/simulation_experiment/results/liosam_rpe_small_house.txt
echo "=== RPE RKO-LIO ===" && grep -E "rmse|mean|median|std" ~/simulation_experiment/results/rkolio_rpe_small_house.txt
```

**Confirmed numbers — small_house_04 (May 20, 2026, high-speed, SUPERSEDED):**

> ⚠️ These numbers are from the first small_house run at unconstrained speed. They are superseded by the `_06` run below which used the correct 0.15 m/s cap. LIO-SAM diverged here because high speed caused scan-matching failures, not environmental difficulty.

| Metric | LIO-SAM | RKO-LIO | Factor (LIO-SAM/RKO-LIO) |
|---|---|---|---|
| APE RMSE (m) | **12.075** | **0.076** | 158× |
| APE Mean (m) | 5.958 | 0.071 | 84× |
| APE Median (m) | 1.213 | 0.067 | 18× |
| APE Max (m) | 40.528 | 0.129 | 314× |
| RPE RMSE (m/m) | **25.233** | **0.030** | 841× |
| RPE Mean (m/m) | 12.932 | 0.026 | 497× |

**FINAL NUMBERS — All three environments at 0.15 m/s (speed-capped, paper-grade):**

Bags: `sim_small_house_06`, `sim_warehouse_06`, `sim_bookstore_01`.

| World | LIO-SAM APE RMSE (m) | RKO-LIO APE RMSE (m) | LIO-SAM RPE RMSE (m/m) | RKO-LIO RPE RMSE (m/m) | Notes |
|---|---|---|---|---|---|
| small_house | **0.049** | 0.148 | **0.025** | 0.042 | LIO-SAM wins; both stable |
| warehouse | **0.040** | 0.163 | **0.023** | 0.033 | LIO-SAM wins; structured aisles favour scan matching |
| bookstore | DIV (21.69) | **0.494** | DIV (22.96) | **0.148** | LIO-SAM **diverged** — estimated path 2632 m vs actual 43 m |

**Detailed numbers per environment:**

*small_house (sim_small_house_06):*
| Metric | LIO-SAM | RKO-LIO |
|---|---|---|
| APE RMSE (m) | 0.049 | 0.148 |
| APE Mean (m) | 0.047 | 0.144 |
| APE Max (m) | 0.090 | 0.237 |
| RPE RMSE (m/m) | 0.025 | 0.042 |
| RPE Mean (m/m) | 0.018 | 0.030 |

*warehouse (sim_warehouse_06):*
| Metric | LIO-SAM | RKO-LIO |
|---|---|---|
| APE RMSE (m) | 0.040 | 0.163 |
| APE Mean (m) | 0.037 | 0.145 |
| APE Max (m) | 0.089 | 0.337 |
| Trajectory length (m) | 42.763 | 47.704 | *(ground truth /odom = 42.178 m)* |
| RPE RMSE (m/m) | 0.023 | 0.033 |
| RPE Mean (m/m) | 0.016 | 0.027 |

*bookstore (sim_bookstore_01):*
| Metric | LIO-SAM | RKO-LIO |
|---|---|---|
| APE RMSE (m) | **21.689** ❌ | **0.494** |
| APE Mean (m) | 16.625 ❌ | 0.434 |
| APE Max (m) | 64.642 ❌ | 0.937 |
| Trajectory length (m) | 2632.130 ❌ | 48.111 | *(ground truth /odom = 43.535 m)* |
| RPE RMSE (m/m) | 22.962 ❌ | 0.148 |
| RPE Mean (m/m) | 16.156 ❌ | 0.056 |

**Key finding for the paper:** LIO-SAM's IMU preintegration + scan matching fails in the bookstore's dense, repetitive retail environment. The estimated path (2632 m) is 60× the actual path (43 m). RKO-LIO's ICP-based registration remained stable throughout. This is the paper's robustness argument: RKO-LIO degrades gracefully; LIO-SAM either succeeds accurately or fails catastrophically.

### Presentation: Slide 12 (Simulation Comparison)

Placeholder type: image + table

| Element | Source |
|---|---|
| Center figure | `screenshots/12_trajectory_overlay_PAPER_FIGURE.png` |
| Table III values | From evo stats files above |
| Qualitative comment | Write based on visual comparison of screenshots 07 and 08 |

### Presentation: Other slides needing real content

| Slide | Placeholder | Source |
|---|---|---|
| 6 (Hardware Platform) | TurtleBot photo | Paper Fig. 1 (real robot photo from real-robot experiments) |
| 7 (System Architecture) | RKO-LIO block diagram | Paper Fig. 2 |
| 10 (Qualitative Results) | 4 maps: basement, corridor, tunnel, LIO-SAM failure | Paper Fig. 4a–4d |
| 14 (Future Work) | Personal future direction bullet | Anas's choice |

### Screenshot checklist (from RUNBOOK.md, consolidated here)

| # | When to take | Content | Filename |
|---|---|---|---|
| 06 | After bag recording | `ros2 bag info` output showing duration + message counts | `06_bag_info.png` |
| 07 | During LIO-SAM replay | RViz showing LIO-SAM map building | `07_liosam_map.png` |
| 08 | During RKO-LIO replay | RViz showing RKO-LIO map building (same view angle as 07) | `08_rkolio_map.png` |
| 09 | After evo APE | LIO-SAM APE plot window | `09_liosam_ape.png` |
| 10 | After evo APE | RKO-LIO APE plot window | `10_rkolio_ape.png` |
| 11 | After `evo_res` | Side-by-side comparison plot | `11_comparison_ape.png` |
| 12 | After `evo_traj` | Trajectory overlay — **main paper figure** | `12_trajectory_overlay_PAPER_FIGURE.png` |
| 15 | End of experiment | Terminal showing final FINAL_RESULTS.txt | `15_final_numbers.png` |

Take all screenshots with `gnome-screenshot` or the system Print Screen key. Store in `~/simulation_experiment/screenshots/`.

### Final checklist before submitting to IEEE

- [x] Section IV-C written with real numbers (no placeholders) — numbers in Section 23 above
- [x] Table III filled in — see FINAL NUMBERS table in Section 23
- [x] PDF figures generated (APE, RPE, trajectory, comparison for all 3 worlds)
- [x] All evo evaluations complete (14 commands, all 3 environments)
- [ ] Figures embedded in paper at 300 DPI minimum (use PDF versions)
- [ ] All slides updated (7 placeholders filled)
- [ ] Author names: Anas Alqadhi, Aysegül Uçar, Bajhaw, Munef
- [ ] Paper ID 331 correct on title slide
- [ ] Date May 22, 2026 correct
- [ ] Backup PDF exported: `libreoffice --headless --convert-to pdf ICHORA2026_RKO_LIO_Presentation.pptx`
- [ ] Practice run completed at least once

---

## 24. Multi-environment experimental plan

This section is the step-by-step plan for repeating Phases 6–9 for the `warehouse` and `bookstore` worlds.
The small_house run (suffix `_04`) is the template — follow the same protocol exactly.

---

### Environment 2: `warehouse`

#### Step 1 — Source the workspace (every new terminal)

```bash
source /opt/ros/humble/setup.bash && source ~/tb3_3d_ws/install/setup.bash
```

#### Step 2 — Phase 6: Record bag in warehouse

**Terminal A — Launch Gazebo + robot (no SLAM):**
```bash
ros2 launch ~/simulation_experiment/configs/record_sim_only.launch.py world:=warehouse
```
Wait 60–90 s for: `Successfully spawned entity [custom_turtlebot_rkolio]`
(warehouse takes longer than small_house — 27 AWS models but heavier physics)

**Terminal B — Record bag:**
```bash
cd ~/simulation_experiment/bags && \
ros2 bag record \
  /velodyne_points /mavros/imu/data /odom /clock \
  -o sim_warehouse_01
```

**Terminal C — Teleop:**
```bash
ros2 launch turtlebot_rkolio_sim teleop.launch.py
```
Drive for at least 3 minutes. Cover all aisles. Return near start (give LIO-SAM a loop closure chance).
Speed: max 0.15 m/s linear, 0.5 rad/s angular.

**Stop:** Ctrl+C in B first, then A. Verify bag:
```bash
ros2 bag info ~/simulation_experiment/bags/sim_warehouse_01/
```
Need: Duration > 180 s, `/velodyne_points` > 400 msgs (Gazebo may throttle), `/mavros/imu/data` > 4000 msgs.

#### Step 3 — Phase 7: Replay through LIO-SAM

**Terminal A — Launch LIO-SAM + RViz:**
```bash
ros2 launch ~/simulation_experiment/configs/replay_liosam.launch.py
```
Wait 10 s for all 4 LIO-SAM nodes to initialize.

**Terminal B — Record LIO-SAM output:**
```bash
cd ~/simulation_experiment/bags && \
ros2 bag record /lio_sam/mapping/odometry /lio_sam/mapping/cloud_registered \
  -o liosam_output_warehouse_01
```

**Terminal C — Play the bag:**
```bash
ros2 bag play ~/simulation_experiment/bags/sim_warehouse_01/ \
  --clock --rate 0.5 \
  --topics /velodyne_points /mavros/imu/data /clock
```
Wait for `Reached end of file`, then Ctrl+C in B then A.

#### Step 4 — Phase 8: Replay through RKO-LIO

**Terminal A — Launch RKO-LIO + RViz:**
```bash
ros2 launch ~/simulation_experiment/configs/replay_rkolio.launch.py
```

**Terminal B — Record RKO-LIO output:**
```bash
cd ~/simulation_experiment/bags && \
ros2 bag record /rko_lio/odometry /rko_lio/local_map \
  -o rkolio_output_warehouse_01
```

**Terminal C — Play the bag:**
```bash
ros2 bag play ~/simulation_experiment/bags/sim_warehouse_01/ \
  --clock --rate 0.5 \
  --topics /velodyne_points /mavros/imu/data /clock
```
Wait for `Reached end of file`, then Ctrl+C in B then A.

#### Step 5 — Merge bags (add GT `/odom` to SLAM output bags)

```bash
python3 ~/simulation_experiment/merge_bags.py \
  ~/simulation_experiment/bags/sim_warehouse_01/ \
  ~/simulation_experiment/bags/liosam_output_warehouse_01/ \
  ~/simulation_experiment/bags/liosam_merged_warehouse_01/

python3 ~/simulation_experiment/merge_bags.py \
  ~/simulation_experiment/bags/sim_warehouse_01/ \
  ~/simulation_experiment/bags/rkolio_output_warehouse_01/ \
  ~/simulation_experiment/bags/rkolio_merged_warehouse_01/
```

#### Step 6 — Phase 9: evo evaluation

```bash
export PATH=$HOME/.local/bin:$PATH
export MPLBACKEND=Agg
cd ~/simulation_experiment/results

# APE LIO-SAM
evo_ape bag2 ~/simulation_experiment/bags/liosam_merged_warehouse_01/ \
  /odom /lio_sam/mapping/odometry \
  --align --save_results liosam_ape_warehouse.zip \
  --save_plot liosam_ape_warehouse.pdf --plot_mode xy \
  2>&1 | tee liosam_ape_warehouse.txt

# APE RKO-LIO
evo_ape bag2 ~/simulation_experiment/bags/rkolio_merged_warehouse_01/ \
  /odom /rko_lio/odometry \
  --align --save_results rkolio_ape_warehouse.zip \
  --save_plot rkolio_ape_warehouse.pdf --plot_mode xy \
  2>&1 | tee rkolio_ape_warehouse.txt

# Comparison bar chart
evo_res liosam_ape_warehouse.zip rkolio_ape_warehouse.zip \
  --use_filenames --save_plot comparison_ape_warehouse.pdf

# Trajectory overlays
evo_traj bag2 ~/simulation_experiment/bags/liosam_merged_warehouse_01/ \
  /odom /lio_sam/mapping/odometry \
  --ref /odom --align --plot_mode xy --save_plot liosam_trajectory_warehouse.pdf

evo_traj bag2 ~/simulation_experiment/bags/rkolio_merged_warehouse_01/ \
  /odom /rko_lio/odometry \
  --ref /odom --align --plot_mode xy --save_plot rkolio_trajectory_warehouse.pdf

# RPE
evo_rpe bag2 ~/simulation_experiment/bags/liosam_merged_warehouse_01/ \
  /odom /lio_sam/mapping/odometry \
  --align --delta 1 --delta_unit m \
  --save_results liosam_rpe_warehouse.zip --save_plot liosam_rpe_warehouse.pdf \
  2>&1 | tee liosam_rpe_warehouse.txt

evo_rpe bag2 ~/simulation_experiment/bags/rkolio_merged_warehouse_01/ \
  /odom /rko_lio/odometry \
  --align --delta 1 --delta_unit m \
  --save_results rkolio_rpe_warehouse.zip --save_plot rkolio_rpe_warehouse.pdf \
  2>&1 | tee rkolio_rpe_warehouse.txt
```

#### Step 7 — IEEE trajectory overlay figure

```bash
cd ~/simulation_experiment
python3 plot_trajectory_overlay.py  # edit BAG paths at top of file to warehouse bags first
```
Edit `plot_trajectory_overlay.py`: change `LIOSAM_BAG` and `RKOLIO_BAG` to the warehouse merged bags, and `TOPIC_LIOSAM`/`TOPIC_RKOLIO` remain the same. Change output filenames to `*_warehouse.pdf/.png`.

#### Step 8 — Extract numbers

```bash
echo "=== warehouse — APE LIO-SAM ==="  && grep -E "rmse|mean|median" ~/simulation_experiment/results/liosam_ape_warehouse.txt
echo "=== warehouse — APE RKO-LIO ==="  && grep -E "rmse|mean|median" ~/simulation_experiment/results/rkolio_ape_warehouse.txt
echo "=== warehouse — RPE LIO-SAM ==="  && grep -E "rmse|mean|median" ~/simulation_experiment/results/liosam_rpe_warehouse.txt
echo "=== warehouse — RPE RKO-LIO ==="  && grep -E "rmse|mean|median" ~/simulation_experiment/results/rkolio_rpe_warehouse.txt
```

---

### Environment 3: `bookstore`

Repeat Steps 1–8 above, replacing every `warehouse` with `bookstore` and every `_warehouse_01` with `_bookstore_01`.

**bookstore-specific notes:**
- 141 AWS models — heaviest world. Gazebo may take **2–3 minutes** to fully load. Wait for the spawn confirmation before recording.
- Expect significant Gazebo throttling (possibly < 50% real-time). The bag replay at `--rate 0.5` should compensate.
- Narrow aisles: drive very slowly (0.10 m/s) to avoid bumping shelves, which creates collision impulses that corrupt IMU data.
- Topics, launch files, and evo commands are identical — just swap the world name and bag suffix.

**Commands summary (bookstore):**
```bash
# Phase 6
ros2 launch ~/simulation_experiment/configs/record_sim_only.launch.py world:=bookstore
ros2 bag record /velodyne_points /mavros/imu/data /odom /clock -o sim_bookstore_01

# Phase 7
ros2 launch ~/simulation_experiment/configs/replay_liosam.launch.py
ros2 bag record /lio_sam/mapping/odometry /lio_sam/mapping/cloud_registered -o liosam_output_bookstore_01
ros2 bag play ~/simulation_experiment/bags/sim_bookstore_01/ --clock --rate 0.5 \
  --topics /velodyne_points /mavros/imu/data /clock

# Phase 8
ros2 launch ~/simulation_experiment/configs/replay_rkolio.launch.py
ros2 bag record /rko_lio/odometry /rko_lio/local_map -o rkolio_output_bookstore_01
ros2 bag play ~/simulation_experiment/bags/sim_bookstore_01/ --clock --rate 0.5 \
  --topics /velodyne_points /mavros/imu/data /clock

# Merge
python3 ~/simulation_experiment/merge_bags.py \
  ~/simulation_experiment/bags/sim_bookstore_01/ \
  ~/simulation_experiment/bags/liosam_output_bookstore_01/ \
  ~/simulation_experiment/bags/liosam_merged_bookstore_01/
python3 ~/simulation_experiment/merge_bags.py \
  ~/simulation_experiment/bags/sim_bookstore_01/ \
  ~/simulation_experiment/bags/rkolio_output_bookstore_01/ \
  ~/simulation_experiment/bags/rkolio_merged_bookstore_01/

# Phase 9 — same evo commands, replace 'warehouse' with 'bookstore' everywhere
```

---

### Kill everything (any time)

```bash
pkill -9 -f "gzserver|gzclient|rko_lio|lio_sam|rviz2|rosbag2|robot_state_publisher"
```

---

### Progress tracker

**Updated (May 20, 2026):** All three environments have been fully completed at the correct speed (0.15 m/s linear / 0.5 rad/s angular) matching real robot conditions. Teleop is hard-capped via `teleop.launch.py`. Bag suffixes used: `_06` for small_house and warehouse (earlier attempts `_05` had issues), `_01` for bookstore.

| Environment | Phase 6 (bag) | Phase 7 (LIO-SAM replay) | Phase 8 (RKO-LIO replay) | Phase 9 (evo eval) | PDF figures |
|-------------|:---:|:---:|:---:|:---:|:---:|
| small_house | ✅ `sim_small_house_06` (833 s) | ✅ `liosam_output_small_house_06` | ✅ `rkolio_output_small_house_06` | ✅ all metrics | ✅ generated |
| warehouse | ✅ `sim_warehouse_06` (626 s) | ✅ `liosam_output_warehouse_06` (1249 s, 1354 msgs) | ✅ `rkolio_output_warehouse_06` (1251 s) | ✅ all metrics | ✅ generated |
| bookstore | ✅ `sim_bookstore_01` (1470 s) | ✅ `liosam_output_bookstore_01` (2933 s, 783 msgs — diverged) | ✅ `rkolio_output_bookstore_01` (2938 s, 2648 msgs) | ✅ all metrics | ✅ generated |

**Merged bags** (source + SLAM output merged for evo single-bag mode):
- `liosam_merged_small_house_06/`, `rkolio_merged_small_house_06/`
- `liosam_merged_warehouse_06/`, `rkolio_merged_warehouse_06/`
- `liosam_merged_bookstore_01/`, `rkolio_merged_bookstore_01/`

**Generated PDF figures** in `~/simulation_experiment/results/`:
- `liosam_ape_<world>.pdf`, `rkolio_ape_<world>.pdf` — individual APE plots
- `comparison_ape_<world>.pdf` — side-by-side comparison
- `liosam_trajectory_<world>.pdf`, `rkolio_trajectory_<world>.pdf` — trajectory overlays
- `liosam_rpe_<world>.pdf`, `rkolio_rpe_<world>.pdf` — RPE plots

---

End of developer guide. Last updated: May 20, 2026 (Session 2 — all experiments complete). Keep EXPERIMENT_LOG.md as the append-only session journal alongside this file.

**Experiment status: COMPLETE.** All three environments done. All evo evaluations done. All PDF figures generated. Paper table ready.
