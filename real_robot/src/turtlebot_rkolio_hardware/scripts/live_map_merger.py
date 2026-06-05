#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import time

class LiveMapMerger(Node):
    def __init__(self):
        super().__init__('live_map_merger')
        
        # Publisher for merged map
        self.merged_map_pub = self.create_publisher(
            PointCloud2, '/live_merged_map', 1)
        
        # Subscriber for local robot map
        self.local_sub = self.create_subscription(
            PointCloud2, '/rko_lio/local_map', 
            self.local_callback, 1)
        
        # Storage for maps
        self.local_map = None
        self.drone_map = None
        
        # Try to subscribe to drone map (may not be available initially)
        self.drone_sub = None
        self.setup_drone_subscription()
        
        # Timer for merging
        self.merge_timer = self.create_timer(1.0, self.merge_and_publish)
        
        self.get_logger().info('Live map merger started')
        
    def setup_drone_subscription(self):
        """Setup drone subscription when available"""
        try:
            self.drone_sub = self.create_subscription(
                PointCloud2, '/rko_lio/local_map', 
                self.drone_callback, 1)
        except:
            self.get_logger().warn('Drone subscription not available yet')
        
    def local_callback(self, msg):
        self.local_map = msg
        
    def drone_callback(self, msg):
        # This will receive drone data when available
        self.drone_map = msg
        self.get_logger().info('Received drone map data')
        
    def merge_and_publish(self):
        if self.local_map is None:
            return
            
        # For now, just republish local map
        # When drone data is available, merge both
        merged = self.local_map
        merged.header.frame_id = 'map'
        merged.header.stamp = self.get_clock().now().to_msg()
        
        self.merged_map_pub.publish(merged)

def main():
    rclpy.init()
    merger = LiveMapMerger()
    print("Live map merger running...")
    rclpy.spin(merger)
    merger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
