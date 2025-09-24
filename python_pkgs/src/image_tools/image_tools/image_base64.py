import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import base64

class ZEDImageBase64Publisher(Node):
    def __init__(self):
        super().__init__('image_base64')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/perception/segmentation/image/compressed',
            self.listener_callback,
            qos_profile_sensor_data   # <-- key change
        )
        self.publisher = self.create_publisher(String, '/perception/segmentation/image/compressed64', 10)

    def listener_callback(self, msg):
        self.get_logger().info(f"Got frame: {len(msg.data)} bytes")
        b64_image = base64.b64encode(msg.data).decode('utf-8')
        self.publisher.publish(String(data=b64_image))

def main(args=None):
    rclpy.init(args=args)
    node = ZEDImageBase64Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
