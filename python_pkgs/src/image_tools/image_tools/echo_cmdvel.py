import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Sub(Node):
    def __init__(self):
        super().__init__('sub_cmd_vel')
        self.create_subscription(Twist, '/cmd_vel', self.cb, 10)

    def cb(self, msg):
        self.get_logger().info(f"Received: {msg.linear.x}")

rclpy.init()
node = Sub()
rclpy.spin(node)
