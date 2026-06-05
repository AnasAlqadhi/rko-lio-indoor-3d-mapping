#!/usr/bin/env python3
"""Republish IMU with system (ROS) time to fix Pixhawk clock offset."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuRepub(Node):
    def __init__(self):
        super().__init__('imu_repub')
        # Publisher uses sensor_data QoS (BEST_EFFORT) — matches rko_lio_node subscription
        self.pub = self.create_publisher(Imu, '/imu/synced', qos_profile_sensor_data)
        # Subscription uses sensor_data QoS (BEST_EFFORT) — required to match MAVROS publisher
        self.sub = self.create_subscription(Imu, '/mavros/imu/data', self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(ImuRepub())


if __name__ == '__main__':
    main()
