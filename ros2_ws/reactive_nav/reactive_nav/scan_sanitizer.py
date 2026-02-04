#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanSanitizer(Node):
    def __init__(self):
        super().__init__("scan_sanitizer")
        self.declare_parameter("in_topic", "/scan")
        self.declare_parameter("out_topic", "/scan_clean")

        in_topic = self.get_parameter("in_topic").value
        out_topic = self.get_parameter("out_topic").value

        self.pub = self.create_publisher(LaserScan, out_topic, 10)
        self.sub = self.create_subscription(LaserScan, in_topic, self.cb, 10)

        self.get_logger().info(f"ScanSanitizer: {in_topic} -> {out_topic}")

    def cb(self, msg: LaserScan):
        # copy message
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.intensities = list(msg.intensities)

        clean = []
        for r in msg.ranges:
            # Isaac is sending -1.0 for "no return"
            if r is None or math.isnan(r) or r <= 0.0:
                clean.append(float("inf"))
            else:
                clean.append(r)
        out.ranges = clean

        self.pub.publish(out)

def main():
    rclpy.init()
    node = ScanSanitizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
