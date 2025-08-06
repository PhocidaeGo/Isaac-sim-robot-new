#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

class OdomTransformNode : public rclcpp::Node {
public:
  OdomTransformNode()
  : Node("odom_transform_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    // Subscriber to original odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/zed/zed_node/odom", 10,
      std::bind(&OdomTransformNode::odomCallback, this, std::placeholders::_1));

    // Publisher to transformed odometry
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/zed/zed_node/odom_map", 10);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::PoseStamped pose_in;
    pose_in.header = msg->header;
    pose_in.pose = msg->pose.pose;

    // Transform from 'odom' to 'map'
    geometry_msgs::msg::PoseStamped pose_out;
    try {
      pose_out = tf_buffer_.transform(pose_in, "map", tf2::durationFromSec(0.1));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform odometry: %s", ex.what());
      return;
    }

    // Create new odometry message
    auto new_odom = *msg;
    new_odom.header.frame_id = "map";
    new_odom.pose.pose = pose_out.pose;

    // child_frame_id remains unchanged
    odom_pub_->publish(new_odom);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTransformNode>());
  rclcpp::shutdown();
  return 0;
}
