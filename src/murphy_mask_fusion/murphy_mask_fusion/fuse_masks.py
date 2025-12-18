import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class MaskFusion(Node):
    def __init__(self):
        super().__init__('mask_fusion')
        self.bridge = CvBridge()

        self.vision = None 
        self.lidar = None 

        self.sub_v = self.create_subscription(
            Image,
            '/murphy/vision/mask',
            self.v_cb,
            10
        )

        self.sub_l = self.create_subscription(
            Image,
            '/murphy/lidar/mask',
            self.l_cb,
            10
        )

        self.pub = self.create_publisher(
            Image,
            '/murphy/vision/fused_mask',
            10
        )

        self.get_logger().info("Mask fusion started")

    def v_cb(self, msg):
        self.vision = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        self.try_pub()

    def l_cb(self, msg):
        self.lidar = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        self.try_pub()

    def try_pub(self):
        if self.vision is None or self.lidar is None:
            return

        v = self.vision
        l = self.lidar

        if v.shape != l.shape:
            v = cv2.resize(
                v,
                (l.shape[1], l.shape[0]),
                interpolation=cv2.INTER_NEAREST
            )

        fused = cv2.bitwise_or(v, l)

        out = self.bridge.cv2_to_imgmsg(fused, encoding='mono8')
        self.pub.publish(out)

def main():
    rclpy.init()
    node = MaskFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
