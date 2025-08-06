/*
help me modify the code, so that the algorithm will scan all the clusters one by one:
1. After first clicked point sent by user, robot starts to observe the cluster.
2. Once the cluster is closed (closed_received_ == true), then publish the point belongs to next cluster 'next_wall_' to /clicked_point to initiate new round scanning.
3. Keep scanning new cluster until cannot find new unclosed cluster  
*/

#include <memory>
#include <atomic>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

enum class PlannerState { IDLE, MOVING_TO_OBSERVE, MOVING_TO_WORK };

class WallPlanner : public rclcpp::Node
{
public:
  WallPlanner()
  : Node("wall_planner_node"),
    state_(PlannerState::IDLE),
    moving_(false),
    robot_busy_(false),
    cloud_received_(false),
    clicked_received_(false),
    normal_received_(false),
    closed_received_(false),
    unclosed_received_(false),
    next_wall_received_(false)
  {
    // Subscribers
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&WallPlanner::cmdVelCallback, this, std::placeholders::_1));

    clustered_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/clustered_cloud_rgb", 10, std::bind(&WallPlanner::cloudCallback, this, std::placeholders::_1));

    clicked_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10, std::bind(&WallPlanner::clickedCallback, this, std::placeholders::_1));

    endpoint_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/cluster_info/endpoint", 10, std::bind(&WallPlanner::endpointCallback, this, std::placeholders::_1));

    normal_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      "/cluster_info/normal", 10, std::bind(&WallPlanner::normalCallback, this, std::placeholders::_1));

    cluster_closed_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/cluster_info/cluster_closed", 10, std::bind(&WallPlanner::closedCallback, this, std::placeholders::_1));

    unclosed_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/cluster_info/unclosed_point", 10, std::bind(&WallPlanner::unclosedCallback, this, std::placeholders::_1));

    next_wall_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/cluster_info/next_wall", 10, std::bind(&WallPlanner::nextWallCallback, this, std::placeholders::_1));

    zed_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/zed/zed_node/pose", 10, std::bind(&WallPlanner::zedPoseCallback, this, std::placeholders::_1));

    working_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/robot_state/working", 10, std::bind(&WallPlanner::workingCallback, this, std::placeholders::_1));

    // Publishers
    way_point_pub_     = create_publisher<geometry_msgs::msg::PointStamped>("/way_point", 10);
    working_pub_       = create_publisher<std_msgs::msg::Bool>("/robot_state/working", 10);
    clicked_point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/clicked_point", 10);

    // Main loop timer
    timer_ = create_wall_timer(100ms, std::bind(&WallPlanner::mainLoop, this));

    RCLCPP_INFO(get_logger(), "WallPlanner node started");
  }

private:
  // Callbacks
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double lin = std::hypot(msg->linear.x, msg->linear.y);
    double ang = std::abs(msg->angular.z);
    moving_ = (lin > 1e-3 || ang > 1e-3);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    cloud_msg_ = *msg;
    cloud_received_ = true;
  }

  void clickedCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    clicked_point_ = *msg;
    clicked_received_ = true;
    closed_received_ = false;
    cloud_received_ = false;
    RCLCPP_INFO(get_logger(), "Started scanning cluster at (%.2f, %.2f, %.2f)",
                msg->point.x, msg->point.y, msg->point.z);
  }

  void endpointCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    endpoints_ = msg->poses;
  }

  void normalCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    normal_ = *msg;
    normal_received_ = true;
  }

  void closedCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    cluster_closed_ = msg->data;
    closed_received_ = true;
    if (cluster_closed_) {
      RCLCPP_INFO(get_logger(), "Cluster closed at (%.2f, %.2f, %.2f)",
                  clicked_point_.point.x, clicked_point_.point.y, clicked_point_.point.z);
    }
  }

  void unclosedCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    unclosed_point_ = *msg;
    unclosed_received_ = true;
  }

  void zedPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
    pose_received_ = true;
    // Check if close to last waypoint
    if (last_waypoint_.header.stamp.sec != 0 && distanceToWaypoint() < 0.5) {
      reach_wp_ = true;
    }
  }

  void nextWallCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    next_wall_ = *msg;
    next_wall_received_ = true;
    // RCLCPP_INFO(get_logger(), "Received next cluster start point (%.2f, %.2f, %.2f)", msg->point.x, msg->point.y, msg->point.z);
  }

  void workingCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    robot_busy_ = msg->data;
  }

  // Main planning loop
  void mainLoop() {
    if (moving_) return;

    // Handle arrival at previous waypoint
    if (reach_wp_) {
      if (state_ == PlannerState::MOVING_TO_OBSERVE || state_ == PlannerState::MOVING_TO_WORK) {
        RCLCPP_INFO(get_logger(), "Reached waypoint via ZED: distance = %.2f", distanceToWaypoint());
        publishWorking(true);
        state_ = PlannerState::IDLE;
        return;
      }
    }

    // Waiting for initial click
    if (!clicked_received_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for initial clicked point...");
      return;
    }

    // No cloud: observe
    if (!cloud_received_ || cloudIsEmpty()) {
      if (!normal_received_ || !unclosed_received_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for normal and unclosed point...");
        return;
      }
      auto wp = unclosed_point_;
      wp.point.x += 2.0f * normal_.x;
      wp.point.y += 2.0f * normal_.y;
      wp.point.z += 2.0f * normal_.z;
      publishWaypoint(wp);
      publishWorking(false);
      RCLCPP_INFO(get_logger(), "Published observe waypoint");
      state_ = PlannerState::MOVING_TO_OBSERVE;
      return;
    }

    // Cloud present: check closure
    if (!closed_received_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for cluster_closed info...");
      return;
    }

    if (cluster_closed_) {
      // Completed this cluster
      if (next_wall_received_) {
        // Initiate next cluster
        next_wall_.header.stamp = get_clock()->now();
        clicked_point_pub_->publish(next_wall_);
        clicked_received_ = false;
        normal_received_ = false;
        unclosed_received_ = false;
        cloud_received_ = false;
        closed_received_ = false;
        next_wall_received_ = false;
        RCLCPP_INFO(get_logger(), "Initiating scan of next cluster");
      } else {
        RCLCPP_INFO(get_logger(), "No more clusters to scan. Done.");
      }
      return;
    }

    // Cluster still open: continue observe
    if (!normal_received_ || !unclosed_received_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for normal and unclosed point...");
      return;
    }
    auto wp = unclosed_point_;
    wp.point.x += 2.0f * normal_.x;
    wp.point.y += 2.0f * normal_.y;
    wp.point.z += 2.0f * normal_.z;
    publishWaypoint(wp);
    publishWorking(false);
    RCLCPP_INFO(get_logger(), "Published observe waypoint");
    state_ = PlannerState::MOVING_TO_OBSERVE;
  }

  // Utilities
  bool cloudIsEmpty() const {
    return cloud_msg_.width * cloud_msg_.height == 0;
  }

  void publishWaypoint(const geometry_msgs::msg::PointStamped &wp) {
    last_waypoint_ = wp;
    way_point_pub_->publish(wp);
  }

  void publishWorking(bool w) {
    std_msgs::msg::Bool msg; msg.data = w;
    working_pub_->publish(msg);
  }

  double distanceToWaypoint() const {
    const auto &p = current_pose_.pose.position;
    const auto &w = last_waypoint_.point;
    return std::hypot(p.x - w.x, p.y - w.y, p.z - w.z);
  }

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr clustered_cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr endpoint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr normal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cluster_closed_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr unclosed_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr next_wall_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr zed_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr working_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr way_point_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr working_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Internal state
  PlannerState state_;
  std::atomic<bool> moving_;
  bool robot_busy_;
  bool cloud_received_, clicked_received_, normal_received_, closed_received_, unclosed_received_, next_wall_received_, reach_wp_, pose_received_;

  sensor_msgs::msg::PointCloud2 cloud_msg_;
  geometry_msgs::msg::PointStamped clicked_point_, unclosed_point_, next_wall_, last_waypoint_;
  geometry_msgs::msg::Vector3 normal_;
  geometry_msgs::msg::PoseStamped current_pose_;
  bool cluster_closed_;
  std::vector<geometry_msgs::msg::Pose> endpoints_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallPlanner>());
  rclcpp::shutdown();
  return 0;
}

