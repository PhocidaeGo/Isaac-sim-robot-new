import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import cv2
from datetime import datetime

class MultiImageSaver(Node):
    def __init__(self):
        super().__init__('multi_image_saver')
        self.bridge = CvBridge()

        # Topics to subscribe to
        self.topics = ['/cam0', '/cam1', '/cam2', '/cam3']

        # Per-topic state: last save time and subfolder
        self.last_saved_time = {}
        self._subs = []

        for topic in self.topics:
            subfolder = topic.strip('/')
            os.makedirs(f'saved_images/{subfolder}', exist_ok=True)
            self.last_saved_time[topic] = self.get_clock().now()

            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, t=topic: self.image_callback(msg, t),
                10)
            self._subs.append(sub)


    def image_callback(self, msg, topic_name):
        now = self.get_clock().now()
        elapsed = (now - self.last_saved_time[topic_name]).nanoseconds / 1e9

        if elapsed < 1.5:
            return

        self.last_saved_time[topic_name] = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f'saved_images/{topic_name.strip("/")}/image_{timestamp}.jpg'
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f"[{topic_name}] Saved: {filename}")
        except Exception as e:
            self.get_logger().error(f"Error processing image from {topic_name}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MultiImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
