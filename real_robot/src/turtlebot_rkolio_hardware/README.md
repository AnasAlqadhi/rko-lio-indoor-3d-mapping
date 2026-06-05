# turtlebot_rkolio_hardware

Real-robot bringup for the custom TurtleBot3 Waffle Pi (Velodyne VLP-16 + Pixhawk Cube Orange+). This package owns the sensor drivers, MAVROS bridge, static TFs, and the per-SLAM configs tuned for the actual hardware.

This package **never runs in simulation.** For Gazebo simulation, see [`turtlebot_rkolio_sim`](../turtlebot_rkolio_sim/).

## Contents

```
turtlebot_rkolio_hardware/
├── config/
│   ├── rkolio_params.yaml          # RKO-LIO real-robot config (proven, 50 Hz IMU tuned)
│   ├── velodyne_config.yaml        # Velodyne driver config
│   ├── velodyne_driver.yaml        # Velodyne driver low-level
│   ├── px4_config.yaml             # PX4 / MAVROS connection settings
│   ├── stable_config.yaml          # Conservative RKO-LIO tuning (drift-safe)
│   ├── lightweight_config.yaml     # Lower-cost tuning (battery / CPU constrained)
│   └── mapping_view.rviz           # RViz layout for live mapping view
├── launch/
│   └── 1_real_robot_bringup.launch.py   # Velodyne + MAVROS + static TFs
├── rviz/
│   └── mapping_view.rviz
└── scripts/                        # Bash + Python helpers for robot operation
    ├── setup_robot_environment.sh
    ├── setup_robot_network.sh
    ├── complete_robot_startup.sh
    ├── simple_robot_start.sh
    ├── start_sensors.sh
    ├── start_px4_sensors.sh
    ├── start_rko_lio.sh
    ├── start_px4_rko_lio.sh
    ├── start_live_dual_system.sh
    ├── start_rviz_only.sh
    ├── imu_repub.py                 # IMU republisher (frame_id + covariance fixups)
    ├── live_map_merger.py           # Merge incremental local maps into a global map
    ├── record_mapping.sh            # rosbag2 recording (current workspace)
    ├── record_mapping_safe.sh       # Recording with explicit topic whitelist
    ├── record_to_desktop.sh         # Record to ~/Desktop (debug)
    ├── playback_mapping.sh          # Replay a recorded bag with RKO-LIO
    └── visualize_recording.sh       # Open RViz against a replayed bag
```

## Hardware

| Component | Spec |
|---|---|
| Robot base | TurtleBot3 Waffle Pi (OpenCR + dual Dynamixel) |
| Compute | NVIDIA Jetson AGX Orin 32 GB |
| LiDAR | Velodyne VLP-16 (10 Hz, 100 m range, 360° × 30°) |
| IMU / FCU | Pixhawk Cube Orange+, IMU @ ~50 Hz via MAVROS |
| Network | Velodyne on `192.168.4.201`, Jetson on `192.168.4.x` |

## Sensor topology

| | Topic | Rate | Frame |
|---|---|---|---|
| LiDAR | `/velodyne_points` | 10 Hz | `velodyne` |
| IMU | `/mavros/imu/data` | ~50 Hz | `imu_link` |

These match the simulation exactly, so SLAM configs are interchangeable in terms of topics — only the *tuning values* differ (e.g. `deskew: true` on real, `deskew: false` in sim because the sim LiDAR plugin lacks per-point timestamps).

## Bringup

### 1. Connect the hardware

- Power the robot, the Velodyne, and the Pixhawk.
- Ethernet to the Velodyne (`192.168.4.201`, port `2368`).
- USB to the Pixhawk (`/dev/ttyACM0` at 921600 baud).

Verify network:
```bash
ping 192.168.4.201
ls /dev/ttyACM0
```

### 2. Source the workspace

```bash
cd ~/tb3_3d_ws
source install/setup.bash
export ROS_DOMAIN_ID=30   # or whatever your fleet uses
```

### 3. Sensor bringup

```bash
ros2 launch turtlebot_rkolio_hardware 1_real_robot_bringup.launch.py
```

This starts:
- Velodyne driver → `/velodyne_points`
- MAVROS (PX4 mode) → `/mavros/imu/data`
- Static TFs `base_link → velodyne` and `base_link → imu_link`

Sanity check the IMU rate (should be ~50 Hz):

```bash
ros2 topic hz /mavros/imu/data
ros2 topic hz /velodyne_points   # ~10 Hz
```

### 4. Start RKO-LIO

```bash
ros2 launch rko_lio odometry.launch.py \
  config_file:=$(ros2 pkg prefix turtlebot_rkolio_hardware)/share/turtlebot_rkolio_hardware/config/rkolio_params.yaml
```

Outputs:
- `/rko_lio/odometry` — pose at LiDAR rate
- `/rko_lio/local_map` — sliding-window voxel map at 1 Hz
- `/rko_lio/frame` — deskewed point cloud per scan
- `/rko_lio/lidar_acceleration` — IMU-frame acceleration estimates

## Configs

### `rkolio_params.yaml` — primary

The proven real-robot config. Tuned for 50 Hz Pixhawk IMU + VLP-16 indoors on a TurtleBot3:

| Parameter | Value | Notes |
|---|---|---|
| `imu_topic` | `/mavros/imu/data` | Standard MAVROS IMU output |
| `lidar_topic` | `/velodyne_points` | Velodyne driver output |
| `deskew` | `true` | Real Velodyne driver emits per-point timestamps |
| `voxel_size` | 0.3 | Trades density for runtime |
| `max_range` | 50 m | VLP-16 spec is 100 m; 50 m is safer indoors |
| `min_range` | 0.9 m | Skips chassis returns |
| `min_beta` | 150 | More IMU trust than the 200 default (Pixhawk EKF is clean) |
| `max_expected_jerk` | 5.0 m/s³ | Filters 50 Hz discretization noise |
| `initialization_phase` | `true` | Estimate gravity & biases on startup |

### `stable_config.yaml` — drift-safe variant

For when accuracy matters more than density. Increases `voxel_size` and `min_beta`.

### `lightweight_config.yaml` — low-cost variant

For when the Jetson is under thermal/power pressure. Drops `max_num_threads`, raises `voxel_size`, halves `max_iterations`.

### `velodyne_config.yaml` / `velodyne_driver.yaml`

Velodyne ROS driver config — IP, port, RPM (600 → 10 Hz). Edit if your Velodyne is on a different network.

### `px4_config.yaml`

MAVROS connection params. Edit `fcu_url` if your Pixhawk is on a different serial port.

## Helper scripts

The `scripts/` directory has shell helpers for common operations. Most assume you've sourced the workspace.

| Script | Purpose |
|---|---|
| `setup_robot_environment.sh` | Source ROS, set `ROS_DOMAIN_ID`, set network |
| `setup_robot_network.sh` | Configure routes for the Velodyne subnet |
| `complete_robot_startup.sh` | Full sequence: env → network → MAVROS → Velodyne → RKO-LIO |
| `simple_robot_start.sh` | Minimal sequence (no MAVROS health checks) |
| `start_sensors.sh` | Just the Velodyne driver |
| `start_px4_sensors.sh` | Velodyne + MAVROS |
| `start_rko_lio.sh [--rviz]` | RKO-LIO with optional RViz |
| `start_px4_rko_lio.sh` | MAVROS + RKO-LIO together |
| `start_live_dual_system.sh` | Sensors + SLAM + live map merger |
| `imu_repub.py` | Sanitizes IMU covariance + frame_id (relay node) |
| `live_map_merger.py` | Merges `/rko_lio/local_map` snapshots into a persistent global map |
| `record_mapping.sh` | rosbag2 recording for offline replay |
| `playback_mapping.sh <bag>` | Replay a bag through RKO-LIO |

### ⚠️ Path correction needed in scripts

Several scripts still reference `~/rko_lio_ws/` (a previous workspace name). The current workspace is `~/tb3_3d_ws/`. Either:

1. Update each script:
   ```bash
   sed -i 's|~/rko_lio_ws|~/tb3_3d_ws|g' scripts/*.sh
   sed -i 's|\$HOME/rko_lio_ws|\$HOME/tb3_3d_ws|g' scripts/*.sh
   ```
2. Or symlink: `ln -s ~/tb3_3d_ws ~/rko_lio_ws`

## IMU upsampling (optional, for LIO-SAM)

LIO-SAM's preintegration is designed for 100–500 Hz IMU. At 50 Hz the optimizer drifts. If you want to run LIO-SAM on the real robot, extend [`scripts/imu_repub.py`](scripts/imu_repub.py) to interpolate 50 Hz Pixhawk samples up to ~200 Hz:

```python
# Pseudocode — interpolate between consecutive Pixhawk samples
# at fixed 200 Hz, publish on /mavros/imu/data_upsampled, and point
# LIO-SAM's imuTopic at that.
```

RKO-LIO and FAST-LIO do **not** need this — they work directly with 50 Hz.

## Recording for offline analysis

```bash
./scripts/record_mapping.sh    # records /velodyne_points, /mavros/imu/data, /tf, /odom
```

Replay later in a sim-like setup:

```bash
ros2 bag play <bag_dir> --clock
ros2 launch rko_lio odometry.launch.py config_file:=... use_sim_time:=true
```

## Troubleshooting

### `/mavros/imu/data` not publishing

- Pixhawk on wrong port? Try `/dev/ttyUSB0` instead of `/dev/ttyACM0`
- Baud rate mismatch — Pixhawk firmware defaults to 921600 for the TELEM ports, 115200 for USB
- Check `ros2 topic list | grep mavros` — if no `/mavros/state`, MAVROS isn't talking to the FCU

### IMU rate is unstable / not 50 Hz

- The Pixhawk's `IMU_HARMONIC` and `INS_RAW_LOG_OPT` parameters affect output rate
- In QGroundControl, set `MAV_0_RATE` for the IMU stream
- Check there's no USB-to-serial contention with QGC running in parallel

### `/velodyne_points` empty

- `ping 192.168.4.201` — is the LiDAR reachable?
- Check the Jetson's route: `ip route get 192.168.4.201`
- The Velodyne sometimes needs a power cycle if it was disconnected mid-stream

### RKO-LIO odometry drifts heavily

- Check the IMU is mounted with **Z up** — if the Pixhawk is sideways, the extrinsics in `rkolio_params.yaml` need updating
- Lower `voxel_size` to 0.2 for more density (at higher CPU cost)
- Try `stable_config.yaml` for a more conservative profile
