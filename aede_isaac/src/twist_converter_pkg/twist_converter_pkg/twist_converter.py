import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist

class TwistConverter(Node):
    def __init__(self):
        super().__init__('twist_converter')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            TwistStamped,
            '/cmd_vel_stamped',
            self.listener_callback,
            10)

    def listener_callback(self, msg: TwistStamped):
        twist_msg = Twist()
        twist_msg.linear = msg.twist.linear
        twist_msg.angular = msg.twist.angular
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
