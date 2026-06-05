#!/usr/bin/env python3
"""
Adds 'ring' and 'time' fields to the Gazebo ray-sensor point cloud.

Gazebo's libgazebo_ros_ray_sensor publishes only x/y/z/intensity.
FAST-LIO and LIO-SAM both need:
  ring  (uint16)  — which of the 16 VLP-16 beams this point came from
  time  (float32) — per-point time offset within the scan [seconds]

Subscribes:  /velodyne_points        (raw from Gazebo)
Publishes:   /velodyne_points_proc   (with ring + time, for FAST-LIO / LIO-SAM)

RKO-LIO reads /velodyne_points directly and does not need this node.
"""

import math
import struct
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# VLP-16: 16 beams, uniform 2-degree spacing, -15 to +15 degrees
VLP16_ANGLES_RAD = np.deg2rad(np.arange(-15, 16, 2, dtype=np.float32))  # 16 values
SCAN_PERIOD = 0.1   # 10 Hz


class Vlp16Fixer(Node):
    def __init__(self):
        super().__init__('vlp16_ring_time_fixer')
        qos = QoSProfile(depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.sub = self.create_subscription(PointCloud2, '/velodyne_points', self._cb, qos)
        self.pub = self.create_publisher(PointCloud2, '/velodyne_points_proc', qos)
        self.get_logger().info(
            'vlp16_ring_time_fixer ready: /velodyne_points → /velodyne_points_proc')

    def _cb(self, msg: PointCloud2):
        # Find field offsets
        field_map = {f.name: f.offset for f in msg.fields}
        if 'x' not in field_map:
            return

        step = msg.point_step
        n = msg.width * msg.height
        data = np.frombuffer(bytes(msg.data), dtype=np.uint8)

        # Extract x, y, z using stride tricks (fast)
        def get_field(offset):
            idx = np.arange(n) * step + offset
            return np.frombuffer(
                np.concatenate([data[i:i+4] for i in idx]).tobytes(),
                dtype=np.float32)

        # Build full data array at once for speed
        raw = data.reshape(n, step)

        x_off = field_map['x']
        y_off = field_map['y']
        z_off = field_map['z']
        i_off = field_map.get('intensity', None)

        xs = raw[:, x_off:x_off+4].copy().view(np.float32).reshape(-1)
        ys = raw[:, y_off:y_off+4].copy().view(np.float32).reshape(-1)
        zs = raw[:, z_off:z_off+4].copy().view(np.float32).reshape(-1)
        if i_off is not None:
            ins = raw[:, i_off:i_off+4].copy().view(np.float32).reshape(-1)
        else:
            ins = np.zeros(n, np.float32)

        # Ring: elevation angle → nearest VLP-16 beam
        r_xy = np.sqrt(xs*xs + ys*ys)
        elev = np.arctan2(zs, np.where(r_xy > 1e-6, r_xy, 1e-6))
        # diff shape: (n, 16), argmin over axis=1
        rings = np.argmin(np.abs(elev[:, None] - VLP16_ANGLES_RAD[None, :]), axis=1).astype(np.uint16)

        # Time: azimuth fraction of scan period
        az = np.arctan2(-ys, xs)           # [-pi, pi]
        az = np.where(az < 0, az + 2*math.pi, az)   # [0, 2pi]
        times = (az / (2 * math.pi) * SCAN_PERIOD).astype(np.float32)

        # Pack output: x(4) y(4) z(4) intensity(4) ring(2) pad(2) time(4) = 24 bytes
        new_step = 24
        out = np.zeros((n, new_step), dtype=np.uint8)
        out[:, 0:4]   = xs.view(np.uint8).reshape(-1, 4)
        out[:, 4:8]   = ys.view(np.uint8).reshape(-1, 4)
        out[:, 8:12]  = zs.view(np.uint8).reshape(-1, 4)
        out[:, 12:16] = ins.view(np.uint8).reshape(-1, 4)
        out[:, 16:18] = rings.view(np.uint8).reshape(-1, 2)
        # bytes 18-19: padding (zeros)
        out[:, 20:24] = times.view(np.uint8).reshape(-1, 4)

        new_msg = PointCloud2()
        new_msg.header = msg.header
        new_msg.height = 1
        new_msg.width = n
        new_msg.fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring',      offset=16, datatype=PointField.UINT16,  count=1),
            PointField(name='time',      offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        new_msg.is_bigendian = False
        new_msg.point_step = new_step
        new_msg.row_step = n * new_step
        new_msg.is_dense = True
        new_msg.data = out.tobytes()
        self.pub.publish(new_msg)


def main():
    rclpy.init()
    node = Vlp16Fixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
