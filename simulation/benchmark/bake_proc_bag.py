#!/usr/bin/env python3
"""
Offline ring/time fixer + bag baker.

Reads a sim bag, replays the EXACT logic of vlp16_ring_time_fixer.py over every
/velodyne_points scan (no real-time pressure -> zero dropped scans), and writes a
new bag containing:
    /velodyne_points_proc  (x,y,z,intensity,ring,time)   -- baked
    /mavros/imu/data       (passthrough)
    /odom                  (passthrough, ground truth)

The baked bag can then be replayed straight into FAST-LIO / LIO-SAM with no live
fixer node, so the Python fixer is no longer a real-time bottleneck.

Usage:
    python3 bake_proc_bag.py <source_bag_dir> <output_bag_dir>
"""

import sys
import math
import numpy as np

import rosbag2_py
from rclpy.serialization import serialize_message, deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2, PointField

VLP16_ANGLES_RAD = np.deg2rad(np.arange(-15, 16, 2, dtype=np.float32))  # 16 beams
SCAN_PERIOD = 0.1  # 10 Hz

PASSTHROUGH = {"/mavros/imu/data", "/odom"}
RAW_TOPIC = "/velodyne_points"
PROC_TOPIC = "/velodyne_points_proc"


def fix_cloud(msg: PointCloud2) -> PointCloud2:
    """Identical math to vlp16_ring_time_fixer.py, applied offline."""
    field_map = {f.name: f.offset for f in msg.fields}
    if "x" not in field_map:
        return None
    step = msg.point_step
    n = msg.width * msg.height
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    raw = data.reshape(n, step)

    x_off, y_off, z_off = field_map["x"], field_map["y"], field_map["z"]
    i_off = field_map.get("intensity", None)
    xs = raw[:, x_off:x_off + 4].copy().view(np.float32).reshape(-1)
    ys = raw[:, y_off:y_off + 4].copy().view(np.float32).reshape(-1)
    zs = raw[:, z_off:z_off + 4].copy().view(np.float32).reshape(-1)
    ins = (raw[:, i_off:i_off + 4].copy().view(np.float32).reshape(-1)
           if i_off is not None else np.zeros(n, np.float32))

    r_xy = np.sqrt(xs * xs + ys * ys)
    elev = np.arctan2(zs, np.where(r_xy > 1e-6, r_xy, 1e-6))
    rings = np.argmin(np.abs(elev[:, None] - VLP16_ANGLES_RAD[None, :]), axis=1).astype(np.uint16)

    az = np.arctan2(-ys, xs)
    az = np.where(az < 0, az + 2 * math.pi, az)
    times = (az / (2 * math.pi) * SCAN_PERIOD).astype(np.float32)

    new_step = 24
    out = np.zeros((n, new_step), dtype=np.uint8)
    out[:, 0:4] = xs.view(np.uint8).reshape(-1, 4)
    out[:, 4:8] = ys.view(np.uint8).reshape(-1, 4)
    out[:, 8:12] = zs.view(np.uint8).reshape(-1, 4)
    out[:, 12:16] = ins.view(np.uint8).reshape(-1, 4)
    out[:, 16:18] = rings.view(np.uint8).reshape(-1, 2)
    out[:, 20:24] = times.view(np.uint8).reshape(-1, 4)

    proc = PointCloud2()
    proc.header = msg.header
    proc.height = 1
    proc.width = n
    proc.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
        PointField(name="time", offset=20, datatype=PointField.FLOAT32, count=1),
    ]
    proc.is_bigendian = False
    proc.point_step = new_step
    proc.row_step = n * new_step
    proc.is_dense = True
    proc.data = out.tobytes()
    return proc


def main():
    src, dst = sys.argv[1], sys.argv[2]

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=src, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=dst, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    # output topics
    writer.create_topic(rosbag2_py.TopicMetadata(
        name=PROC_TOPIC, type="sensor_msgs/msg/PointCloud2", serialization_format="cdr"))
    for t in PASSTHROUGH:
        if t in topic_types:
            writer.create_topic(rosbag2_py.TopicMetadata(
                name=t, type=topic_types[t], serialization_format="cdr"))

    n_proc = n_pass = 0
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic == RAW_TOPIC:
            msg = deserialize_message(data, PointCloud2)
            proc = fix_cloud(msg)
            if proc is not None:
                writer.write(PROC_TOPIC, serialize_message(proc), ts)
                n_proc += 1
        elif topic in PASSTHROUGH:
            writer.write(topic, data, ts)  # raw bytes passthrough (no deserialize)
            n_pass += 1

    print(f"baked {n_proc} proc clouds, {n_pass} passthrough msgs -> {dst}")


if __name__ == "__main__":
    main()
