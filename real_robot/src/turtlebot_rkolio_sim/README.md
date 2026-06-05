# turtlebot_rkolio_sim

Gazebo Classic 11 simulation of the custom TurtleBot3 Waffle Pi for 3D LIO evaluation. This package owns the simulated robot model, the worlds, and the per-SLAM launch files that wire everything together (Gazebo + robot + a chosen SLAM + RViz).

This package **never runs on the real robot.** For real-robot deployment, see [`turtlebot_rkolio_hardware`](../turtlebot_rkolio_hardware/).

## Contents

```
turtlebot_rkolio_sim/
├── config/
│   ├── sim_rkolio_config.yaml      # RKO-LIO sim-specific config
│   └── sim_rviz.rviz                # RViz layout for sim
├── launch/
│   ├── simulation.launch.py         # Gazebo + robot only (no SLAM)
│   ├── sim_with_rkolio.launch.py    # Full stack: Gazebo + robot + RKO-LIO + RViz
│   ├── sim_with_fastlio.launch.py   # Full stack: Gazebo + robot + FAST-LIO + RViz
│   ├── sim_with_liosam.launch.py    # Full stack: Gazebo + robot + LIO-SAM + RViz
│   └── teleop.launch.py             # teleop_twist_keyboard in xterm
├── models/
│   └── custom_turtlebot_rkolio/     # Gazebo SDF model (used for spawning)
├── urdf/
│   ├── custom_turtlebot.urdf.xacro  # URDF used by robot_state_publisher (TF tree)
│   └── sensors.urdf.xacro            # LiDAR + IMU sensor macros
├── scripts/
│   ├── run_sim.sh                    # Convenience wrapper
│   └── record_sim.sh                 # rosbag2 recording helper
└── worlds/
    ├── corridor.world
    ├── basement_room.world
    ├── tunnel.world
    ├── static_arena.world
    └── turtlebot3_world.world
```

## Robot model

The simulated robot is a TurtleBot3 Waffle Pi with three extra layers:

1. **Jetson AGX Orin** housing (black box on top of the base plate)
2. **Pixhawk Cube Orange+** with simulated IMU (the orange cube)
3. **Velodyne VLP-16** on top with custom mount bracket

The Pixhawk Cube houses the `imu_link` frame — i.e. the simulated IMU sensor sits inside the cube, exactly where it does on the real robot.

### Sensor publishing

| Sensor | Topic | Rate | Frame | Plugin |
|---|---|---|---|---|
| LiDAR (VLP-16) | `/velodyne_points` | 10 Hz | `velodyne` | `gazebo_ros_ray_sensor` (CPU ray, 900×16 = 14,400 points/scan) |
| IMU (Pixhawk) | `/mavros/imu/data` | 50 Hz | `imu_link` | `gazebo_ros_imu_sensor` |
| Wheel odometry | `/odom` | 30 Hz | `odom`→`base_footprint` | `gazebo_ros_diff_drive` |
| Camera (if used) | `/camera/image_raw`, `/camera/camera_info` | — | — | (from SDF) |

The IMU is named `/mavros/imu/data` to match the real Pixhawk → MAVROS pipeline so SLAM configs are identical between sim and real.

## Usage

### Just the sim (no SLAM)

```bash
ros2 launch turtlebot_rkolio_sim simulation.launch.py world:=corridor
```

Useful for debugging the world or recording rosbags from sensor topics.

### Sim + a SLAM system

Pick the SLAM you want to evaluate:

```bash
# RKO-LIO — primary, proven on the real robot
ros2 launch turtlebot_rkolio_sim sim_with_rkolio.launch.py world:=corridor

# FAST-LIO — reference iEKF LIO
ros2 launch turtlebot_rkolio_sim sim_with_fastlio.launch.py world:=corridor

# LIO-SAM — reference factor-graph LIO with loop closure
ros2 launch turtlebot_rkolio_sim sim_with_liosam.launch.py world:=corridor
```

The launchers all share the same Gazebo setup, the same robot, the same teleop. They differ only in which SLAM nodes get started.

### Driving the robot

Open a second terminal:

```bash
source ~/tb3_3d_ws/install/setup.bash
ros2 launch turtlebot_rkolio_sim teleop.launch.py
```

This opens an xterm with `teleop_twist_keyboard`. Standard `i / j / l / , / k` keys.

### Worlds

Pick a world with `world:=<name>`:

| World | Description | Best for |
|---|---|---|
| `corridor` (default) | Indoor corridor with side rooms | SLAM drift measurement |
| `basement_room` | Single rectangular room | Loop closure stress test |
| `tunnel` | Narrow tunnel | Thin-feature challenges |
| `static_arena` | Large open area + scattered objects | Open-space SLAM |
| `turtlebot3_world` | Stock TB3 world | Sanity checks |
| `house` | Full house — rooms, kitchen, hallways | Residential indoor mapping |
| `small_house` | AWS residential house (detailed) | Realistic home SLAM |
| `bookstore` | AWS bookstore — bookshelves, desks | Cluttered indoor SLAM |
| `warehouse` | AWS small warehouse — shelves, pallets | Industrial SLAM |
| `warehouse_no_roof` | Same warehouse, no ceiling | Better LiDAR visibility |
| `willowgarage` | Willow Garage office floor plan (classic ROS) | Multi-room office SLAM |
| `cafe` | Indoor café | Semi-open indoor SLAM |
| `dqn_stage1` | Simple obstacles | Easy testing |
| `dqn_stage2` | Medium obstacles | Medium difficulty |
| `dqn_stage3` | Complex obstacles | Hard testing |
| `dqn_stage4` | Most complex obstacles | Maximum stress test |

**Model paths** — all world models are pre-installed in `~/.gazebo/models` (AWS models) and `/opt/ros/humble/share/turtlebot3_gazebo/models` (TB3 house). Both paths are set in `~/.bashrc` via `GAZEBO_MODEL_PATH` and injected by the launchers at startup.

## Configs

### RKO-LIO sim config — [`config/sim_rkolio_config.yaml`](config/sim_rkolio_config.yaml)

Mirrors the real-robot config in `turtlebot_rkolio_hardware/config/rkolio_params.yaml` with one difference:

- `deskew: false` in sim — the Gazebo `gazebo_ros_ray_sensor` plugin does not emit per-point timestamps, which RKO-LIO needs for motion compensation. On the real robot the Velodyne driver emits per-point timestamps, so `deskew: true` works there.

### FAST-LIO / LIO-SAM configs

Live with the respective upstream packages. Sim-specific overrides are in:

- [`src/FAST_LIO/config/turtlebot3_vlp16_sim.yaml`](../FAST_LIO/config/turtlebot3_vlp16_sim.yaml)
- [`src/LIO-SAM/config/turtlebot3_vlp16_sim.yaml`](../LIO-SAM/config/turtlebot3_vlp16_sim.yaml)

Both configs are tuned for 50 Hz IMU (raised process noise covariances vs. their defaults).

## Troubleshooting

### `/velodyne_points` has no data

Gazebo Classic's sensor plugins occasionally fail to initialize on busy systems. The plugin loads (you see `gazebo_ros_velodyne_laser: ready` in the log) but the topic stays silent.

**Fix:** Ctrl-C the launch and restart it. If it keeps happening, lower the LiDAR sample count in [`urdf/sensors.urdf.xacro`](urdf/sensors.urdf.xacro) from 1800 to 900.

### RKO-LIO logs "Skipping IMU, waiting for first LiDAR message"

The SLAM has IMU but no LiDAR yet. Usually a startup race — wait 5–10 s. If it persists, see the previous troubleshooting entry (LiDAR not publishing).

### Robot doesn't move when you publish `/cmd_vel`

Check `/joint_states` — if the wheel velocities are non-zero but `/odom` stays at zero, the wheels are slipping (likely a collision issue with the world geometry). Try a different world.

### Gazebo real-time factor very low (< 0.5)

The CPU ray sensor is expensive. Drop `<samples>` in `urdf/sensors.urdf.xacro` from 1800 to 900 or 360. Sim will run faster but with sparser scans.

## Recording for offline replay

```bash
./scripts/record_sim.sh   # records /velodyne_points, /mavros/imu/data, /tf, /odom, /clock
```

Replay later with `ros2 bag play <bag_dir> --clock` and launch any SLAM separately.
