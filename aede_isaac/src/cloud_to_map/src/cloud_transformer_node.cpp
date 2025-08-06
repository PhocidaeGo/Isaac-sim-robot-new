#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

class CloudTransformerNode : public rclcpp::Node {
public:
    CloudTransformerNode() : Node("cloud_transformer_node") {
        // TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publisher
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/zed/zed_node/point_cloud/cloud_registered_map", 10);

        // Subscriber
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/zed/zed_node/point_cloud/cloud_registered", 10,
            std::bind(&CloudTransformerNode::pointCloudCallback, this, std::placeholders::_1));
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        try {
            // Lookup transform to 'map' frame
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_->lookupTransform("map", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.2));

            sensor_msgs::msg::PointCloud2 transformed_cloud;
            tf2::doTransform(*msg, transformed_cloud, transform);

            // Update frame_id to 'map' before publishing
            transformed_cloud.header.frame_id = "map";

            cloud_pub_->publish(transformed_cloud);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudTransformerNode>());
    rclcpp::shutdown();
    return 0;
}
