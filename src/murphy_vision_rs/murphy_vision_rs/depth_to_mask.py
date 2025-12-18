import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class DepthToMask(Node):
    def __init__(self):
        super().__init__('depth_to_mask')
        self.bridge = CvBridge()

        self.declare_parameter('max_range', 3.0)
        self.max_range = self.get_parameter('max_range').value

        self.sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.cb,
            10
        )

        self.pub = self.create_publisher(
            Image,
            '/murphy/vision/mask',
            10
        )

        self.get_logger().info('RealSense depth → mask started')

    def cb(self, msg):
        # Convert depth (uint16, mm)
        depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_m = depth_mm.astype(np.float32) * 0.001

        # Binary mask
        mask = np.zeros(depth_m.shape, dtype=np.uint8)
        mask[(depth_m > 0.3) & (depth_m < self.max_range)] = 255

        # Publish mono8
        out = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        out.header = msg.header
        self.pub.publish(out)


def main():
    rclpy.init()
    node = DepthToMask()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

