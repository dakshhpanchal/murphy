import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import numpy as np
import math
import cv2

class ScanToMask(Node):
    def __init__(self):
        super().__init__('scan_to_mask')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.cb,
            10
        )

        self.pub = self.create_publisher(
            Image,
            '/murphy/lidar/mask',
            10
        )

        self.size = 480
        self.scale = 100.0  # pixels per meter
        self.get_logger().info("RPLIDAR → mask started")

    def cb(self, scan):
        img = np.zeros((self.size, self.size), dtype=np.uint8)
        cx = cy = self.size // 2

        angle = scan.angle_min
        for r in scan.ranges:
            if 0.1 < r < scan.range_max:
                x = int(cx + math.cos(angle) * r * self.scale)
                y = int(cy - math.sin(angle) * r * self.scale)
                if 0 <= x < self.size and 0 <= y < self.size:
                    img[y, x] = 255
            angle += scan.angle_increment

        out = self.bridge.cv2_to_imgmsg(img, encoding='mono8')
        out.header = scan.header
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(ScanToMask())
    rclpy.shutdown()

