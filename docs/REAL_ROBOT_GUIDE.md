# Real-Robot Guide — RKO-LIO Live Mapping

Operating guide for the TurtleBot Waffle Pi + Velodyne VLP-16 + Pixhawk + Jetson
AGX Orin stack. All scripts live in
[`../real_robot/src/turtlebot_rkolio_hardware/scripts/`](../real_robot/src/turtlebot_rkolio_hardware/scripts/).

> The startup scripts call `sudo` to mount the SSD and configure the LiDAR
> Ethernet link. They will **prompt for your device password**. For unattended
> startup, configure passwordless sudo for the specific `mount` and `ip` commands
> (see "Passwordless sudo" below).

---

## 1. Connect to the robot

```bash
ssh rai@<robot_ip>     # robot's current Wi-Fi IP (changes per network)
```
- Find the robot's IP on the robot itself: `hostname -I` / `iwgetid`.
- Over a USB cable, the Jetson is reachable at the fixed gadget IP `192.168.55.1`.
- VNC (for RViz) is available on port `5901`.

## 2. LiDAR auto-detection

The Velodyne VLP-16 is probed at these addresses (first reachable wins):
`192.168.4.201`, `192.168.1.201`, `192.168.8.201`.
The Jetson assigns itself `192.168.8.100/24` on its LiDAR Ethernet interface.

> Some Velodyne units stream UDP (ports 2368/8308) but ignore ping. If ping fails,
> check `/velodyne_packets` and `/velodyne_points` before declaring failure.

## 3. Start live mapping

```bash
# Launch the full stack inside a tmux session (asks whether to record):
bash ~/.../turtlebot_rkolio_hardware/scripts/start_rko_lio.sh

# Watch the stack:
tmux attach -t mapping

# Stop everything:
tmux kill-session -t mapping
```

Launch **without** the recording prompt:
```bash
RECORD_BAG=false bash ~/.../scripts/start_rko_lio.sh
```

## 4. Recording

```bash
# Record to SSD (from the tmux record window):
bash ~/.../scripts/record_bag.sh

# Record a full session + auto LIO-SAM map save (workspace path):
bash ~/.../scripts/record_full.sh my_session
```

## 5. Playback

```bash
bash ~/.../scripts/play_bag.sh        # choose a bag
bash ~/.../scripts/play_last_bag.sh   # most recent bag
```

## 6. Recording paths

| Script | Destination |
|---|---|
| `record_bag.sh` | `/mnt/ssd/recordings/` (SSD) |
| `record_full.sh` | `<workspace>/recordings/` (symlink → SSD) |

> Recordings are **large** and are **never** committed to GitHub. Keep them on the
> SSD / external drive.

## 7. Health check

```bash
source /opt/ros/humble/setup.bash
source ~/<workspace>/install/setup.bash
export ROS_DOMAIN_ID=45
ros2 topic list | grep -E 'velodyne|rko_lio|mavros/imu/data'
ros2 topic hz /velodyne_points     # expect ~10 Hz
ros2 topic hz /rko_lio/odometry    # expect steady output
```

## 8. Convert a bag → bin + poses (for offline / simulation replay)

```bash
python3 ~/.../scripts/extract_rosbag_db3_to_bin_poses.py \
  <bag_folder> <output_folder> \
  --lidar_topic /velodyne_points --odom_topic /rko_lio/odometry
```

---

## Passwordless sudo (optional, for unattended startup)

Add a sudoers drop-in so the startup script never blocks on a password:

```bash
sudo visudo -f /etc/sudoers.d/rkolio
# add (replace <user>):
<user> ALL=(root) NOPASSWD: /usr/bin/mount /dev/nvme0n1p1 /mnt/ssd, \
  /usr/sbin/ip link set eno1 *, /usr/sbin/ip addr add 192.168.8.100/24 dev eno1
```

> Security note: the original scripts contained a hardcoded sudo password. That has
> been removed in this public release. Never commit device passwords to git.
