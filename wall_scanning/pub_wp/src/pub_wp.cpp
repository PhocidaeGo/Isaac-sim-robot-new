#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <vector>
#include <iostream>
#include <chrono>

class WaypointPublisher : public rclcpp::Node
{
public:
    WaypointPublisher()
    : Node("waypoint_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/way_point", 10);
        load_waypoints();
        RCLCPP_INFO(this->get_logger(), "Waypoint publisher started. Press Enter to publish each waypoint.");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&WaypointPublisher::check_input, this));
    }

private:
    void load_waypoints()
    {
        // Add all waypoints here
        std::vector<std::vector<double>> raw_points = {
            {3.370343, -3.294204, -0.000348}, 
            {3.318254, -7.495191, 0.036972}, 
            {0.327247, -7.738260, 0.012837}, 
            {0.547372, -1.595156, 0.053582}, 
            {1.296725, 2.639912, -0.009142}, 
            {2.424797, 6.327585, -0.042051}, 
            {0.908228, -6.610846, -0.090285}, 
            {3.123357, 8.029848, 0.039605}
        };

        for (const auto& p : raw_points)
        {
            geometry_msgs::msg::PointStamped point_msg;
            point_msg.header.frame_id = "map";
            point_msg.point.x = p[0];
            point_msg.point.y = p[1];
            point_msg.point.z = p[2];
            waypoints_.push_back(point_msg);
        }
    }

    void check_input()
    {
        if (std::cin.peek() != EOF)
        {
            std::string input;
            std::getline(std::cin, input);
            if (current_index_ < waypoints_.size())
            {
                auto msg = waypoints_[current_index_];
                msg.header.stamp = this->get_clock()->now();
                publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Published waypoint %ld", current_index_);
                ++current_index_;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "All waypoints published.");
                rclcpp::shutdown();
            }
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
    std::vector<geometry_msgs::msg::PointStamped> waypoints_;
    size_t current_index_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // Enable non-blocking input for Enter key
    std::cin.sync_with_stdio(false);
    std::cin.tie(nullptr);

    rclcpp::spin(std::make_shared<WaypointPublisher>());
    rclcpp::shutdown();
    return 0;
}
