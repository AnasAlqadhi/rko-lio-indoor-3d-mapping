# Hardware Platform

The mapping system is built around a **TurtleBot Waffle Pi** (differential drive)
customized with an extended sensor and compute stack.

## Components (Table I)

| Component | Specification |
|---|---|
| Base platform | TurtleBot Waffle Pi (differential drive) |
| Compute | NVIDIA Jetson AGX Orin, 32 GB, Ubuntu 22.04 |
| LiDAR | Velodyne VLP-16: 16 beams, 10 Hz, 100 m range, ±3 cm |
| IMU | Pixhawk Cube Orange+: 9-DOF, 50 Hz output |
| OS / Middleware | Ubuntu 22.04 LTS + ROS 2 Humble Hawksbill |
| SLAM algorithm | RKO-LIO (kinematic scan-to-map odometry) |
| Housing | Custom SolidWorks frame; 3D-printed mounts |

## Mounting & frames
- The **Velodyne VLP-16** is **top-mounted** for an unobstructed 360° field of view.
- The **Pixhawk Cube Orange+** is **internally mounted** and calibrated against the
  LiDAR frame via an extrinsic transformation.
- Extrinsics are set directly in the RKO-LIO config to avoid a TF lookup in the IMU
  callback (see `real_robot/src/turtlebot_rkolio_hardware/config/`).

## Data rates
- LiDAR point clouds: **10 Hz** (`/velodyne_points`)
- IMU: **50 Hz** (`/mavros/imu/data`)
- Fused odometry + map: produced by RKO-LIO on the Jetson AGX Orin
  (`/rko_lio/odometry`, `/rko_lio/local_map`).

## Networking
- LiDAR is reached over a dedicated Ethernet link; the Jetson assigns itself
  `192.168.8.100/24` and the Velodyne is auto-detected at one of:
  `192.168.4.201`, `192.168.1.201`, `192.168.8.201`.
- Remote access: SSH (terminal) + VNC (RViz2 visualization) over LAN/VPN.
- Multi-robot isolation via ROS 2 `ROS_DOMAIN_ID` separation (this platform uses 45).

> Add wiring / mounting close-up photos to `media/figures/fig9_setup_photos.png`.
