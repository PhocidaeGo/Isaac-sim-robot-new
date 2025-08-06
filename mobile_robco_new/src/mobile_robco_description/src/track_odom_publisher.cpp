#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class TrackOdomPublisher : public rclcpp::Node
{
public:
  TrackOdomPublisher() : Node("track_odom_publisher")
  {
    // Create a publisher for the /track_odom topic.
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/track_odom", 10);
    // Publish at a rate of 10 Hz (adjust the interval as needed).
    timer_ = this->create_wall_timer(8ms, std::bind(&TrackOdomPublisher::publish_odom, this));
  }

private:
  void publish_odom()
  {
    auto odom_msg = nav_msgs::msg::Odometry();
    
    // Set header information
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";       // Frame in which the odometry is reported.
    odom_msg.child_frame_id = "base_link";     // Typically your robot’s base.

    // Zero position (no movement)
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;
    // Identity quaternion for no rotation.
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;

    // Zero velocity
    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    // For pose covariance (example values)
    for (int i = 0; i < 36; ++i) {
        odom_msg.pose.covariance[i] = (i % 7 == 0) ? 0.1 : 0.0;
    }
    // For twist covariance (example values)
    for (int i = 0; i < 36; ++i) {
        odom_msg.twist.covariance[i] = (i % 7 == 0) ? 0.1 : 0.0;
    }

    // Publish the message
    odom_pub_->publish(odom_msg);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrackOdomPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
