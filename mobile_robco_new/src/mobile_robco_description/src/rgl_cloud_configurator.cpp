#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

// Define a structure for our new point type.
// Layout (24 bytes total):
//   - float x;         //  4 bytes
//   - float y;         //  4 bytes
//   - float z;         //  4 bytes
//   - float intensity; //  4 bytes
//   - uint16_t ring;   //  2 bytes
//   - uint16_t pad;    //  2 bytes (padding)
//   - float time;      //  4 bytes
struct PointXYZIR
{
  float x;
  float y;
  float z;
  float intensity;
  uint16_t ring;
  uint16_t pad;   // padding (set to zero)
  float time;
};
static_assert(sizeof(PointXYZIR) == 24, "PointXYZIR must be 24 bytes");

class RGLCloudConfigurator : public rclcpp::Node
{
public:
  // Provide a constructor that accepts NodeOptions so that use_sim_time
  // is applied before the node's clock is created.
  explicit RGLCloudConfigurator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("rgl_cloud_configurator", options),
    min_angle_(-0.7853975f),  // radians
    max_angle_(0.7853975f),   // radians
    n_rings_(64)
  {
    // If use_sim_time is not already declared by external configuration, declare it here.
    if (!this->has_parameter("use_sim_time")) {
      this->declare_parameter("use_sim_time", false);
    }
    bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
    RCLCPP_INFO(this->get_logger(), "use_sim_time: %s", use_sim_time ? "true" : "false");

    // Calculate the angular resolution between rings.
    angle_step_ = (max_angle_ - min_angle_) / (n_rings_ - 1);

    // Create a subscription to the original point cloud.
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/points", 10,
      std::bind(&RGLCloudConfigurator::pointcloud_callback, this, std::placeholders::_1));

    // Create a publisher for the configured point cloud.
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/lidar/points/configured", 10);

    RCLCPP_INFO(this->get_logger(), "RGL Cloud Configurator node started.");
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const size_t num_points = msg->width * msg->height;
    if (num_points == 0) {
      RCLCPP_WARN(this->get_logger(), "Received empty point cloud.");
      return;
    }

    // Create iterators for the original point cloud fields.
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");

    std::vector<PointXYZIR> new_points;
    new_points.reserve(num_points);

    // Process each point.
    for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
      float x = *iter_x;
      float y = *iter_y;
      float z = *iter_z;
      float intensity = *iter_intensity;

      float horizontal_dist = std::sqrt(x * x + y * y);
      float vertical_angle = std::atan2(z, horizontal_dist);

      int ring = static_cast<int>(std::round((vertical_angle - min_angle_) / angle_step_));
      ring = std::clamp(ring, 0, n_rings_ - 1);

      PointXYZIR pt;
      pt.x = x;
      pt.y = y;
      pt.z = z;
      pt.intensity = intensity;
      pt.ring = static_cast<uint16_t>(ring);
      pt.pad = 0;
      pt.time = 0.0f;

      new_points.push_back(pt);
    }

    auto new_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // Copy the header from the incoming message then update the stamp.
    // When use_sim_time is enabled (and /clock is publishing), this will be simulation time.
    new_msg->header = msg->header;
    new_msg->header.stamp = this->get_clock()->now();

    new_msg->height = msg->height;
    new_msg->width = num_points;

    // Define the fields.
    sensor_msgs::msg::PointField field;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;

    field.name = "x"; field.offset = 0;
    new_msg->fields.push_back(field);

    field.name = "y"; field.offset = 4;
    new_msg->fields.push_back(field);

    field.name = "z"; field.offset = 8;
    new_msg->fields.push_back(field);

    field.name = "intensity"; field.offset = 12;
    new_msg->fields.push_back(field);

    field.name = "ring"; field.offset = 16;
    field.datatype = sensor_msgs::msg::PointField::UINT16;
    new_msg->fields.push_back(field);

    field.name = "time"; field.offset = 20;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    new_msg->fields.push_back(field);

    new_msg->is_bigendian = false;
    new_msg->point_step = 24;  // 24 bytes per point.
    new_msg->row_step = new_msg->point_step * new_msg->width;
    new_msg->is_dense = true;
    new_msg->data.resize(num_points * new_msg->point_step);

    // Copy the processed point data into the message's data buffer.
    std::memcpy(new_msg->data.data(), new_points.data(), new_points.size() * sizeof(PointXYZIR));

    RCLCPP_INFO(this->get_logger(), "Publishing configured cloud with %zu points", num_points);
    publisher_->publish(*new_msg);
  }

  // Member variables
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  const float min_angle_;
  const float max_angle_;
  const int n_rings_;
  float angle_step_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Pass NodeOptions with a parameter override so that use_sim_time is set from the start.
  auto options = rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}});
  auto node = std::make_shared<RGLCloudConfigurator>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
