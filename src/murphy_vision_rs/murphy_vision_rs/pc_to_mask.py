import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2
from cv_bridge import CvBridge
import tf2_ros
import tf2_py

class PCToMask(Node):
    def __init__(self):
        super().__init__('pc_to_mask')
        self.bridge = CvBridge()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.cb,
            10
        )

        self.pub = self.create_publisher(
            Image,
            '/murphy/vision/mask',
            10
        )

        self.size = 480
        self.scale = 20.0  # pixels per meter

    def cb(self, msg):
        mask = np.zeros((self.size, self.size), dtype=np.uint8)

        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                rclpy.time.Time()
            )
        except Exception:
            return

        for p in pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True):
            x, y, z = p

            # ground filtering
            if z < 0.1 or z > 2.5:
                continue

            # robot-centric projection
            px = int(self.size/2 + x * self.scale)
            py = int(self.size - y * self.scale)

            if 0 <= px < self.size and 0 <= py < self.size:
                mask[py, px] = 255

        out = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(PCToMask())
    rclpy.shutdown()
