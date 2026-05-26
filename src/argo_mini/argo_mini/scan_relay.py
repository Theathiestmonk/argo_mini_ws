#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay')

        self.declare_parameter('min_range', 0.20)
        self.min_range = self.get_parameter('min_range').value

        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(LaserScan, '/scan_corrected', 10)

        self.get_logger().info(
            f'Scan relay ready — timestamp fix + min_range filter ({self.min_range:.2f} m)')

    def scan_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()

        # Replace readings closer than min_range with inf (no return).
        # Removes permanent close-range returns from LiDAR mount pillars.
        if self.min_range > 0.0:
            msg.range_min = self.min_range
            msg.ranges = tuple(
                r if r >= self.min_range else math.inf
                for r in msg.ranges
            )

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    relay = ScanRelay()
    rclpy.spin(relay)
    relay.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
