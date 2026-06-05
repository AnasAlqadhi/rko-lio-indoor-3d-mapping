# Drift-Resilient Indoor 3D Mapping in GPS-Denied Environments using RKO-LIO

**Velodyne VLP-16 + Pixhawk Cube Orange+ on a TurtleBot Waffle Pi — ROS 2 Humble**

Official code release for the conference paper:

> **Drift-Resilient Indoor 3D Mapping in GPS-Denied Environments Using RKO-LIO with
> Velodyne–Pixhawk Integration in ROS 2**
> Anas Mahyoub Naji Saeed Alqadhi, Munef el Muhammed, Mohammed Ali Mohammed S. Bajhaw,
> Ayşegül Uçar — *Mechatronics Engineering Department, Fırat University, Elazığ, Turkey.*

<!-- TODO: add the platform photo once available -->
<!-- ![Platform](media/figures/fig1_platform.png) -->

---

## Highlights

- 🛰️ **Real-robot 3D mapping** with **RKO-LIO** (sensor-agnostic LiDAR–inertial
  odometry) — centimeter-level accuracy: **RMSE 0.026 m**, mean error **0.69 %**.
- 🧪 **Controlled simulation benchmark** of **RKO-LIO vs LIO-SAM vs FAST-LIO** via
  identical-bag replay in three GPS-denied Gazebo environments.
- 🛡️ **Worst-case robustness, not peak accuracy**, is the decisive property for real
  deployment: RKO-LIO is the only method with bounded error in *every* environment
  (≤ 0.494 m APE) without per-scene tuning.
- ♻️ No time-synchronization "chaotic image" failures (observed with LIO-SAM) —
  **13.7× lower RMSE** than a prior LIO-SAM deployment on identical hardware.

See full numbers in **[docs/RESULTS.md](docs/RESULTS.md)**.

---

## Repository layout

```
rko-lio-indoor-3d-mapping/
├── real_robot/        # ROS 2 packages for the TurtleBot + Velodyne + Pixhawk stack
│   └── src/
│       ├── turtlebot_rkolio_hardware/   # launch, config, scripts (real robot)
│       └── turtlebot_rkolio_sim/        # Gazebo worlds + sim launch/config
├── simulation/        # identical-bag benchmark (RKO-LIO/LIO-SAM/FAST-LIO) + evo
├── third_party/       # RKO-LIO, FAST-LIO, LIO-SAM as git submodules (upstream)
├── docs/              # guides: real robot, simulation, hardware, results
├── media/             # paper figures, demo videos, result tables (CSV)
└── scripts/           # dependency install helpers
```

> ⚠️ **Recordings (rosbags) are NOT included** — they are large and live on external
> storage. The repository ships code, configs, docs, figures, and result tables only.

---

## Quick start

### 1. Clone with submodules
```bash
git clone --recurse-submodules https://github.com/AnasAlqadhi/rko-lio-indoor-3d-mapping.git
cd rko-lio-indoor-3d-mapping
# if you forgot --recurse-submodules:
git submodule update --init --recursive
```

### 2. Build the real-robot workspace (ROS 2 Humble)
```bash
# Put the packages into a colcon workspace (or build in place):
source /opt/ros/humble/setup.bash
cd real_robot
colcon build --symlink-install
source install/setup.bash
```

### 3. Run live mapping on the robot
```bash
bash real_robot/src/turtlebot_rkolio_hardware/scripts/start_rko_lio.sh
tmux attach -t mapping
```
Full operating instructions: **[docs/REAL_ROBOT_GUIDE.md](docs/REAL_ROBOT_GUIDE.md)**.

### 4. Reproduce the simulation benchmark
See **[docs/SIMULATION_GUIDE.md](docs/SIMULATION_GUIDE.md)** and
**[simulation/README.md](simulation/README.md)**.

---

## System

| Component | Specification |
|---|---|
| Base | TurtleBot Waffle Pi (differential drive) |
| Compute | NVIDIA Jetson AGX Orin, 32 GB, Ubuntu 22.04 |
| LiDAR | Velodyne VLP-16 (16 beams, 10 Hz, 100 m) |
| IMU | Pixhawk Cube Orange+ (9-DOF, 50 Hz) |
| Middleware | ROS 2 Humble Hawksbill |
| Odometry | RKO-LIO (kinematic scan-to-map) |

Details: **[docs/HARDWARE.md](docs/HARDWARE.md)**.

---

## Media

- **Figures:** `media/figures/` — see **[media/README.md](media/README.md)** for the
  figure checklist and capture tips.
- **Videos:** real-robot + RViz demos — see **[media/video/PLACEHOLDER_demo.md](media/video/PLACEHOLDER_demo.md)**.
- **Tables:** `media/tables/*.csv` (Table II & III, ready for plotting).

---

## Third-party SLAM systems

This work references **RKO-LIO**, **FAST-LIO**, and **LIO-SAM** as git submodules.
They retain their own upstream licenses. See **[third_party/README.md](third_party/README.md)**.

---

## Citation

If you use this code, please cite the paper (see **[CITATION.cff](CITATION.cff)**):

```bibtex
@inproceedings{alqadhi2026rkolio,
  title     = {Drift-Resilient Indoor 3D Mapping in GPS-Denied Environments Using
               RKO-LIO with Velodyne--Pixhawk Integration in ROS 2},
  author    = {Alqadhi, Anas Mahyoub Naji Saeed and el Muhammed, Munef and
               Bajhaw, Mohammed Ali Mohammed S. and U{\c{c}}ar, Ay{\c{s}}eg{\"u}l},
  year      = {2026}
  % booktitle / doi: to be added upon publication
}
```

## Acknowledgment

Funded by **TÜBİTAK** (grant 123E406) and **Fırat University FÜBAP** (grants MF.24.80,
MF.25154, MF.25155). Part of this work was conducted within the TÜBİTAK 2209 project
and the master's thesis of Munef el Muhammed.

## License

The code authored in this repository is released under the **MIT License** (see
[LICENSE](LICENSE)). Third-party submodules retain their own licenses.
