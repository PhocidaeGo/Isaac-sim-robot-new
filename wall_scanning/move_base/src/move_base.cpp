/*
help me write a C++ ros2 script to plan the move for robot around a wall:

1. subscribe to /cmd_vel, check if robot is moving, if yes, do nothing and return, wait for it becomes static.
2. subscribe topic (PointCloud2) /clustered_cloud_rgb and (PointStamped) /clicked_point.
- check if clustered_cloud_rgb is empty,
  - if yes:
    - check if clicked_point is empty, 
      - if yes, use rclcpp out a message: "Please click a point in point cloud".
      - if not, calculate the first observe_pos for robot. observe_pos is the position of robot base for observeing the wall,
        first observe_pos = clicked_point + 2.0f * normal,
        publish it to way_point, wait for robot gets static (reach the goal). 
  - if not:
    - check if /cluster_info/cluster_closed == true:
      - if yes, calculate the work_pos,
        work_pos = clicked_point + 0.5f * normal
        publish it to way_point, wait for robot gets static (reach the goal). 

      - if not:
        calculate the first observe_pos for robot. subscribe to /unclosed_point, calculate
        observe_pos = unclosed_point + 2.0f * normal
        publish it to way_point, wait for robot gets static (reach the goal). 

        Keep doing this, until /cluster_info/cluster_closed == true. Then calculate work_pos as mentioned


subscribtions: 
1. (PointCloud2) /clustered_cloud_rgb: point cloud of the wall
2. (PointStamped) /clicked_point: a point on the wall
3. (PoseArray) /cluster_info/endpoint: contains two poses, you can only use the positions parts, they are the two endpoints
4. (Vector3) /cluster_info/normal: the normalized normal of wall
5. (Bool) /cluster_info/cluster_closed: tells you if the wall is fully observed.
6. (Bool) /robot_state/working: robot base can move if false.
7. (PointStamped) /cluster_info/unclosed_point: unclosed point

publish:
1. (PointStamped) /way_point: robot goes through these waypoint to reach the observe_pos or work_pos
2. (Bool) /robot_state/working: robot base will not move if true.

Observing first strategy, avoid erros due to vibration during working and texture-poor due to too close to wall
*/

/*
    // Corner detection
    if (closed_pt_received_) {
      auto next_wall = next_wall_;
      
      auto closed_pt = closed_point_;
      auto wp = closed_point_;
      double dx = closed_pt.point.x - next_wall.point.x;
      double dy = closed_pt.point.y - next_wall.point.y;

      // Rotate the vector 45 degrees clockwise (i.e., -45 degrees)
      // Rotation matrix for -θ:
      // [ cos(θ)  sin(θ)]
      // [-sin(θ)  cos(θ)]
      double angle_rad = -M_PI / 4.0;  // -45 degrees in radians
      double cos_theta = std::cos(angle_rad);
      double sin_theta = std::sin(angle_rad);

      double rx = cos_theta * dx + sin_theta * dy;
      double ry = -sin_theta * dx + cos_theta * dy;

      // Normalize
      double magnitude = std::sqrt(rx * rx + ry * ry);
      // Avoid division by zero
      if (magnitude > 1e-6) {
          rx /= magnitude;
          ry /= magnitude;
      } else {
          RCLCPP_WARN(get_logger(), "Attempted to normalize a zero-length vector.");
          rx = 0.0;
          ry = 0.0;
      }

      wp.point.x += (2.0f * normal_.x + 1.0*rx);
      wp.point.y += (2.0f * normal_.y + 1.0*ry);
      wp.point.z = 0.2f;

      publishWaypoint(wp);
      publishWorking(false);
      RCLCPP_INFO(get_logger(), "Published observe waypoint (closed)");
      state_ = PlannerState::MOVING_TO_OBSERVE;
    }
    */

#include <memory>
#include <atomic>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

enum class PlannerState { IDLE, MOVING_TO_OBSERVE, MOVING_TO_WORK };

class WallPlanner : public rclcpp::Node
{
public:
  WallPlanner()
  : Node("move_base_node"),
    state_(PlannerState::IDLE),
    moving_(false),
    scan_finished(false),
    cloud_received_(false),
    clicked_received_(false),
    normal_received_(false),
    closed_received_(false),
    unclosed_received_(false),
    chunk_planning_done_(false),
    chunk_index_(0)
  {
    // Subscribers
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&WallPlanner::cmdVelCallback, this, _1));

    clustered_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/clustered_cloud_rgb", 10, std::bind(&WallPlanner::cloudCallback, this, _1));

    clicked_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10, std::bind(&WallPlanner::clickedCallback, this, _1));

    cluster_info_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/cluster_info/endpoint", 10, std::bind(&WallPlanner::endpointCallback, this, _1));

    normal_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      "/cluster_info/normal", 10, std::bind(&WallPlanner::normalCallback, this, _1));

    cluster_closed_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/cluster_info/cluster_closed", 10, std::bind(&WallPlanner::closedCallback, this, _1));

    unclosed_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/cluster_info/unclosed_point", 10, std::bind(&WallPlanner::unclosedCallback, this, _1));
    
    closed_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/cluster_info/closed_point", 10, std::bind(&WallPlanner::closePtCallback, this, _1));

    next_wall_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/cluster_info/next_wall", 10, std::bind(&WallPlanner::nextWallCallback, this, _1));

    zed_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/zed/zed_node/pose", 10, std::bind(&WallPlanner::zedPoseCallback, this, std::placeholders::_1));

    working_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/robot_state/working", 10, std::bind(&WallPlanner::workingCallback, this, _1));

    // Publishers
    way_point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/way_point", 10);
    working_pub_   = create_publisher<std_msgs::msg::Bool>("/robot_state/working", 10);
    work_positions_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/work_positions", 10);

    // Timer for main loop
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&WallPlanner::mainLoop, this));

    RCLCPP_INFO(get_logger(), "move base node started");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Consider moving if any linear or angular velocity magnitude > small threshold
    double lin = std::hypot(msg->linear.x, msg->linear.y);
    double ang = std::abs(msg->angular.z);
    moving_ = (lin > 1e-3 || ang > 1e-3);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    cloud_msg_ = *msg;
    cloud_received_ = true;
  }

  void clickedCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    clicked_point_ = *msg;
    clicked_received_ = true;
  }

  void endpointCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    endpoints_ = msg->poses;
  }

  void normalCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    normal_ = *msg;
    normal_received_ = true;
  }

  void closedCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    cluster_closed_ = msg->data;
    closed_received_ = true;
  }

  void unclosedCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    unclosed_point_ = *msg;
    unclosed_received_ = true;
  }

  void closePtCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    closed_point_ = *msg;
    closed_pt_received_ = true;
  }

  void nextWallCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    next_wall_ = *msg;
    next_wall_received_ = true;
  }

  void zedPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
    pose_received_ = true;
    // Check if close to last waypoint
    if (last_waypoint_.header.stamp.sec != 0 && distanceToWaypoint() < 0.5) {
      reach_wp_ = true;
    }
  }

  void workingCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    // This flag indicates if robot is currently busy
    robot_busy_ = msg->data;
  }

  void mainLoop()
  {
    if (moving_) { // if (moving_ || robot_busy_)
      // Robot is moving or busy: wait
      return;
    }

    // If we just reached a waypoint
    if (reach_wp_) {
      if (state_ == PlannerState::MOVING_TO_OBSERVE || state_ == PlannerState::MOVING_TO_WORK) {
        RCLCPP_INFO(get_logger(), "Reached waypoint via ZED: distance = %.2f", distanceToWaypoint());
        publishWorking(true);
        state_ = PlannerState::IDLE;
        return;
      }
    }
    

    // Beginning planning
    if (!cloud_received_ || cloudIsEmpty()) {
      // No cloud data
      if (!clicked_received_) {
        RCLCPP_INFO(get_logger(), "Please click a point in point cloud");
        return;
      }
      if (!normal_received_) {
        RCLCPP_INFO(get_logger(), "Waiting for wall normal");
        return;
      }
      // Compute first observe_pos
      auto wp = clicked_point_;
      wp.point.x += 2.0f * normal_.x;
      wp.point.y += 2.0f * normal_.y;
      wp.point.z = 0.2f;

      publishWaypoint(wp);
      publishWorking(false);
      RCLCPP_INFO(get_logger(), "Published observe waypoint (no cloud)");
      state_ = PlannerState::MOVING_TO_OBSERVE;
      return;
    }

    // Cloud not empty
    if (!closed_received_) {
      RCLCPP_INFO(get_logger(), "Waiting for cluster_closed info");
      return;
    }

    // Cloud closed, go to work position
    if (cluster_closed_ && !scan_finished) {
      if (!normal_received_ || !clicked_received_) {
        RCLCPP_INFO(get_logger(), "Waiting for clicked point or normal");
        return;
      }
      // Compute work_pos
      auto wp = clicked_point_;
      wp.point.x += 1.0f * normal_.x;
      wp.point.y += 1.0f * normal_.y;
      wp.point.z = 0.2f;

      publishWaypoint(wp);
      publishWorking(false);
      RCLCPP_INFO(get_logger(), "Published work waypoint (cluster closed)");
      state_ = PlannerState::MOVING_TO_WORK;
      scan_finished = true;
      return;
    }

    if (!normal_received_) {
      RCLCPP_INFO(get_logger(), "Waiting for normal");
      return;
    }
    if (!unclosed_received_) {
      RCLCPP_INFO(get_logger(), "Waiting for unclosed point");
      return;
    }
    
    // TODO: Add a look-around function to check walls for single camera
    auto wp = unclosed_point_;
    // Compute observe_pos for next segment
    // TODO: consider add a bias towards centroid of wall to avoid collision;
    wp.point.x += 2.0f * normal_.x;
    wp.point.y += 2.0f * normal_.y;
    wp.point.z = 0.2f;

    publishWaypoint(wp);
    publishWorking(false);
    RCLCPP_INFO(get_logger(), "Published observe waypoint (unclosed)");
    state_ = PlannerState::MOVING_TO_OBSERVE;

    // Planning for work posistions
    if (scan_finished && !moving_ && !chunk_planning_done_) {
      if (endpoints_.size() < 2) {
        RCLCPP_INFO(get_logger(), "Waiting for two endpoints to calculate chunks");
        return;
      }
      computeChunkWaypoints();
      startPublishingChunks();
      chunk_planning_done_ = true;
    }
  }

  // Compute waypoints chunk-by-chunk along the wall segment
/*
help me add a function and call it in main loop for working chunk by chunk 
1. only call this function if scan_finished=true and moving_ = false;
2. robot still focus on clustered_cloud_rgb and is standing at the first work_pos, which is in front of the clicked_point.
3. subscribe to /cluster_info/endpoint, it contains two poses, you can only use the positions parts, they are the two endpoints. 
4. Use these 2 end points calculate the waypoints: (note, you only need to consider xy coordinates, all z coordinates should be set to 0.2)
start with one end point1, divide the cluster segment into many chunks, each chunk has length = 2m, if the last segment is less than 2m, still keep it. For first segment, the work position is 1m away from the center of this chunk(+ 1.0f * normal_); then you use normalized(endpoint2-endpoint1)*2.0 + last waypoint to get the next. For the segment < 2m, use normalized(endpoint2-endpoint1)*1.0 + last waypoint. 
5. After calculating, publish them one by one to a ros2 topic: /work_positions. time interval=2s
*/
  void computeChunkWaypoints() {
    auto p1 = endpoints_[0].position;
    auto p2 = endpoints_[1].position;
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double length = std::hypot(dx, dy);
    if (length < 1e-6) return;
    double ux = dx / length;
    double uy = dy / length;

    // Determine number of chunks
    int n_chunks = static_cast<int>(std::ceil(length / 2.0));
    double consumed = 0.0;
    chunk_waypoints_.clear();

    for (int i = 0; i < n_chunks; ++i) {
      double seg_len = (i < n_chunks - 1) ? 2.0 : (length - consumed);
      double center_dist = consumed + seg_len / 2.0;
      geometry_msgs::msg::PointStamped wp;
      wp.header.frame_id = clicked_point_.header.frame_id;
      wp.header.stamp = get_clock()->now();
      // XY along segment
      wp.point.x = p1.x + ux * center_dist;
      wp.point.y = p1.y + uy * center_dist;
      // Offset away from wall by normal
      wp.point.x += normal_.x * 1.0;
      wp.point.y += normal_.y * 1.0;
      // Fixed height
      wp.point.z = 0.2;

      chunk_waypoints_.push_back(wp);
      consumed += seg_len;
    }
    chunk_index_ = 0;
    RCLCPP_INFO(get_logger(), "Computed %zu chunk waypoints", chunk_waypoints_.size());
  }

  // Timer callback to publish next chunk every 2s
  void startPublishingChunks() {
    chunk_timer_ = create_wall_timer(
      2s, std::bind(&WallPlanner::publishNextChunk, this));
  }
  void publishNextChunk() {
    if (chunk_index_ < chunk_waypoints_.size()) {
      work_positions_pub_->publish(chunk_waypoints_[chunk_index_]);
      RCLCPP_INFO(get_logger(), "Published work chunk %zu", chunk_index_);
      chunk_index_++;
    } else {
      chunk_timer_->cancel();
      RCLCPP_INFO(get_logger(), "Finished publishing all work chunks");
    }
  }

  // utilities
  bool cloudIsEmpty()
  {
    return cloud_msg_.width * cloud_msg_.height == 0;
  }

  void publishWaypoint(const geometry_msgs::msg::PointStamped &wp)
  {
    last_waypoint_ = wp;
    way_point_pub_->publish(wp);
  }

  void publishWorking(bool working)
  {
    std_msgs::msg::Bool msg;
    msg.data = working;
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
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cluster_info_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr normal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cluster_closed_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr unclosed_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr closed_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr next_wall_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr zed_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr working_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr way_point_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr working_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr work_positions_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr chunk_timer_;

  // Internal state
  PlannerState state_;
  std::atomic<bool> moving_;
  bool robot_busy_{false};

  bool cloud_received_, clicked_received_, normal_received_, closed_received_, unclosed_received_, closed_pt_received_, next_wall_received_, scan_finished, chunk_planning_done_, reach_wp_, pose_received_;
  sensor_msgs::msg::PointCloud2 cloud_msg_;
  geometry_msgs::msg::PointStamped clicked_point_, unclosed_point_, closed_point_, next_wall_, last_waypoint_;
  geometry_msgs::msg::Vector3 normal_;
  geometry_msgs::msg::PoseStamped current_pose_;
  bool cluster_closed_;

  std::vector<geometry_msgs::msg::Pose> endpoints_;
  std::vector<geometry_msgs::msg::PointStamped> chunk_waypoints_;
  size_t chunk_index_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallPlanner>());
  rclcpp::shutdown();
  return 0;
}
