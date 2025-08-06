#include <memory>
#include <vector>
#include <cmath>
#include <unordered_set>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>  // <-- This is required for remove NaNs

#include <pcl/features/boundary.h>

#include <Eigen/Dense>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/ransac.h>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/bool.hpp"



class RegionGrowingSegmentationNode : public rclcpp::Node
{
public:
    RegionGrowingSegmentationNode()
    : Node("region_growing_segmentation_node"), has_click_(false)
    {
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/zed/zed_node/mapping/fused_cloud", 10,
            std::bind(&RegionGrowingSegmentationNode::cloud_callback, this, std::placeholders::_1));

        click_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10,
            std::bind(&RegionGrowingSegmentationNode::click_callback, this, std::placeholders::_1));

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/clustered_cloud_rgb", 10);

        endpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/cluster_info/endpoint", 10);
        normal_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/cluster_info/normal", 10);
        cluster_closed_pub_ = this->create_publisher<std_msgs::msg::Bool>("/cluster_info/cluster_closed", 10);
        unclosed_pt_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/cluster_info/unclosed_point", 10);
        closed_pt_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/cluster_info/closed_point", 10);
        next_wall_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/cluster_info/next_wall", 10);

        RCLCPP_INFO(this->get_logger(), "Region Growing Segmentation Node started.");
    }

private:
    void click_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        last_click_ = *msg;
        has_click_ = true;
        RCLCPP_INFO(this->get_logger(), "Received clicked point: [%.2f, %.2f, %.2f]",
                    last_click_.point.x, last_click_.point.y, last_click_.point.z);
    }

    /*
        Help me add a function, determine if this highlighted cluster is completed:
1. project all points of highlighted cluster to XY plane, fit a line segment use these points, the two endpoints of the line are notated as e_point1 and e_point2. notate the normal of the line as normal_highlight
2. set two cylinder windows for checking the point distribution around these 2 end points, use end point as center, radius=1.5m, height 1m to 1.5m
3. For each endpoint, first filter out points belong to highlighted cluster, record num_filtered_point.  if number of rest point <= 0.6* num_filtered_point, then consider the cluster is not completed, return False.
If num_rest_point > 0.6* num_filtered_point, run RANSAC on these points, if a plane is fitted and the normal of plane is differ to normal_highlight by at leas 60 degree, then this end point is closed. if two end points are closed then the cluster is complete, return True. if not, the cluster is not complete.

Add following modification to the code:
1. use rclcpp show the cause of not completed cluster.
2. if normal of RANSAC plane is differ to normal_highlight by less than 20 degree, then add these point to current cluster, keep do this step until the number of  external points is not enough or the end point is closed    
*/


    // Enhanced completion check with logging and iterative expansion
    // Computes endpoints and normal based on current cluster points
    struct Endpoints {
        Eigen::Vector3f e1;
        Eigen::Vector3f e2;
        Eigen::Vector3f normal;
    };

    Endpoints computeEndpointsAndNormal(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const pcl::PointIndices& cluster_indices)
    {
        std::vector<Eigen::Vector2f> pts2d;
        pts2d.reserve(cluster_indices.indices.size());
        float sum_z = 0.0f;
        for (int idx : cluster_indices.indices) {
            const auto &p = cloud->points[idx];
            pts2d.emplace_back(p.x, p.y);
            sum_z += p.z;
        }
        float mean_z = sum_z / pts2d.size();
        Eigen::Vector2f mean2d = Eigen::Vector2f::Zero();
        for (auto &v : pts2d) mean2d += v;
        mean2d /= pts2d.size();

        Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
        for (auto &v : pts2d) {
            Eigen::Vector2f d = v - mean2d;
            cov += d * d.transpose();
        }
        cov /= (pts2d.size() - 1);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(cov);
        Eigen::Vector2f dir2d = solver.eigenvectors().col(1).normalized();

        float min_proj = std::numeric_limits<float>::max();
        float max_proj = -min_proj;
        for (auto &v : pts2d) {
            float proj = dir2d.dot(v - mean2d);
            min_proj = std::min(min_proj, proj);
            max_proj = std::max(max_proj, proj);
        }

        Eigen::Vector3f center(mean2d.x(), mean2d.y(), mean_z);
        Eigen::Vector3f e1 = center + dir2d.x() * min_proj * Eigen::Vector3f::UnitX()
                             + dir2d.y() * min_proj * Eigen::Vector3f::UnitY();
        Eigen::Vector3f e2 = center + dir2d.x() * max_proj * Eigen::Vector3f::UnitX()
                             + dir2d.y() * max_proj * Eigen::Vector3f::UnitY();
        Eigen::Vector3f normal(dir2d.y(), -dir2d.x(), 0.0f);
        normal.normalize();

        return {e1, e2, normal};
    }

    void publish_unclosed_point(float x, float y, float z)
    {
        geometry_msgs::msg::PointStamped point_msg;
        point_msg.header.stamp = this->get_clock()->now();
        point_msg.header.frame_id = "map";  // or any other frame

        point_msg.point.x = x;
        point_msg.point.y = y;
        point_msg.point.z = z;

        unclosed_pt_pub_->publish(point_msg);
    }

    void publish_closed_point(float x, float y, float z)
    {
        geometry_msgs::msg::PointStamped point_msg;
        point_msg.header.stamp = this->get_clock()->now();
        point_msg.header.frame_id = "map";  // or any other frame

        point_msg.point.x = x;
        point_msg.point.y = y;
        point_msg.point.z = z;

        closed_pt_pub_->publish(point_msg);
    }

    void publish_next_wall(float x, float y, float z)
    {
        geometry_msgs::msg::PointStamped point_msg;
        point_msg.header.stamp = this->get_clock()->now();
        point_msg.header.frame_id = "map";  // or any other frame

        point_msg.point.x = x;
        point_msg.point.y = y;
        point_msg.point.z = z;

        next_wall_pub_->publish(point_msg);
    }

    void publish_endpoints(const Endpoints& cluster_info) {
        // Publish e1 and e2 as PoseArray (only using position fields)
        geometry_msgs::msg::PoseArray pose_array_msg;
        pose_array_msg.header.stamp = this->get_clock()->now();
        pose_array_msg.header.frame_id = "map";  // or your preferred frame

        geometry_msgs::msg::Pose pose1;
        pose1.position.x = cluster_info.e1.x();
        pose1.position.y = cluster_info.e1.y();
        pose1.position.z = cluster_info.e1.z();
        pose_array_msg.poses.push_back(pose1);

        geometry_msgs::msg::Pose pose2;
        pose2.position.x = cluster_info.e2.x();
        pose2.position.y = cluster_info.e2.y();
        pose2.position.z = cluster_info.e2.z();
        pose_array_msg.poses.push_back(pose2);

        endpoint_pub_->publish(pose_array_msg);

        // Publish normal vector
        geometry_msgs::msg::Vector3 normal_msg;
        normal_msg.x = cluster_info.normal.x();
        normal_msg.y = cluster_info.normal.y();
        normal_msg.z = cluster_info.normal.z();
        normal_pub_->publish(normal_msg);
    }

    bool isClusterCompleted(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        pcl::PointIndices& cluster_indices)
    {
        const float radius = 1.5f, min_z = 0.2f, max_z = 1.8f;
        const float angle_close = 65.0f * M_PI / 180.0f;
        const float angle_expand = 25.0f * M_PI / 180.0f;
        const float cos_close = std::cos(angle_close);
        const float cos_expand = std::cos(angle_expand);
        const float threshold_factor = 0.6f;

        int closed_count = 0;
        // Loop over both endpoints
        for (int endpoint_id = 0; endpoint_id < 2; ++endpoint_id) {
            bool endpoint_closed = false;
            while (true) {
                // **Recalculate endpoints & normal at top of loop**
                Endpoints ep = computeEndpointsAndNormal(cloud, cluster_indices);
                Eigen::Vector3f endpoint = (endpoint_id == 0 ? ep.e1 : ep.e2);

                // Collect cluster vs. external points within cylinder at current endpoint
                std::vector<int> cluster_pts, other_pts;
                for (size_t i = 0; i < cloud->points.size(); ++i) {
                    const auto &p = cloud->points[i];
                    float dx = p.x - endpoint.x();
                    float dy = p.y - endpoint.y();
                    float dist_xy = std::hypot(dx, dy);
                    float dz = p.z;
                    if (dist_xy <= radius && dz >= min_z && dz <= max_z) {
                        bool in_cluster = std::binary_search(
                            cluster_indices.indices.begin(), 
                            cluster_indices.indices.end(), i);
                        if (in_cluster) cluster_pts.push_back(i);
                        else other_pts.push_back(i);
                    }
                }
                int num_cluster = cluster_pts.size();
                int num_other = other_pts.size();

                if (num_other <= threshold_factor * num_cluster) {
                    RCLCPP_WARN(get_logger(),
                        "Endpoint %d at (%.2f,%.2f,%.2f): insufficient external points %d <= %.2f*%d",
                        endpoint_id, endpoint.x(), endpoint.y(), endpoint.z(),
                        num_other, threshold_factor, num_cluster);
                    cluster_info = ep;
                    publish_unclosed_point(endpoint.x(), endpoint.y(), endpoint.z());
                    return false;
                }

                if (num_other < 3) {
                    RCLCPP_WARN(get_logger(),
                        "Endpoint %d at (%.2f,%.2f,%.2f): not enough points for RANSAC (%d)",
                        endpoint_id, endpoint.x(), endpoint.y(), endpoint.z(), num_other);
                    cluster_info = ep;
                    publish_unclosed_point(endpoint.x(), endpoint.y(), endpoint.z());
                    return false;
                }

                // RANSAC plane fitting on external points
                auto other_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
                for (int idx : other_pts) other_cloud->points.push_back(cloud->points[idx]);
                pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(
                    new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(other_cloud));
                pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
                ransac.setDistanceThreshold(0.05);
                if (!ransac.computeModel()) {
                    RCLCPP_WARN(get_logger(),
                        "Endpoint %d at (%.2f,%.2f,%.2f): RANSAC failed", 
                        endpoint_id, endpoint.x(), endpoint.y(), endpoint.z());
                    cluster_info = ep;
                    publish_unclosed_point(endpoint.x(), endpoint.y(), endpoint.z());
                    return false;
                }

                Eigen::VectorXf coeff;
                ransac.getModelCoefficients(coeff);
                Eigen::Vector3f plane_normal(coeff[0], coeff[1], coeff[2]);
                plane_normal.normalize();

                float dot = std::abs(plane_normal.dot(ep.normal));
                float angle_deg = std::acos(dot) * 180.0f / M_PI;

                if (dot <= cos_close) {
                    // Endpoint closed
                    RCLCPP_INFO(get_logger(),
                        "Endpoint %d closed: normal deviation %.1f° >= 60°", endpoint_id, angle_deg);
                    endpoint_closed = true;
                    publish_closed_point(endpoint.x(), endpoint.y(), endpoint.z());

                    // Calculate centroid of next potential cluster
                    Eigen::Vector3f next_cluster_centroid(0.0f, 0.0f, 0.0f);
                    int next_cluster_count = 0;
                    for (int idx : other_pts) {
                        const auto& p = cloud->points[idx];
                        next_cluster_centroid += Eigen::Vector3f(p.x, p.y, p.z);
                        ++next_cluster_count;
                    }
                    if (next_cluster_count > 0) {
                        next_cluster_centroid /= static_cast<float>(next_cluster_count);
                        RCLCPP_INFO(get_logger(), 
                            "Next cluster centroid (%.2f, %.2f, %.2f) calculated from %d points", 
                            next_cluster_centroid.x(), next_cluster_centroid.y(), 
                            next_cluster_centroid.z(), next_cluster_count);
                        // Optional: publish this point for visualization
                        publish_next_wall(next_cluster_centroid.x(), next_cluster_centroid.y(), 0.2f);
                    }
                    ep = computeEndpointsAndNormal(cloud, cluster_indices);
                    break;
                } else if (dot >= cos_expand) {
                    // Expand cluster with nearby points
                    RCLCPP_INFO(get_logger(),
                        "Endpoint %d expanding: normal deviation %.1f° <= 20°, adding %d points", 
                        endpoint_id, angle_deg, num_other);
                    // Merge and keep sorted for binary_search
                    cluster_indices.indices.insert(
                        cluster_indices.indices.end(), other_pts.begin(), other_pts.end());
                    std::sort(cluster_indices.indices.begin(), cluster_indices.indices.end());
                    // Loop continues, endpoints will be recalculated
                } else {
                    RCLCPP_WARN(get_logger(),
                        "Endpoint %d at (%.2f,%.2f,%.2f): normal deviation %.1f° not in expand/close range", 
                        endpoint_id, endpoint.x(), endpoint.y(), endpoint.z(), angle_deg);
                    cluster_info = ep;
                    publish_unclosed_point(endpoint.x(), endpoint.y(), endpoint.z());
                    return false;
                }
                cluster_info = ep;
            }
            if (endpoint_closed) ++closed_count;
        }
        return closed_count == 2;
    }


    void publish_cluster_closed(bool is_closed) {
        std_msgs::msg::Bool msg;
        msg.data = is_closed;
        cluster_closed_pub_->publish(msg);
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Only process once we have a clicked point
        if (!has_click_) {
            return;
        }

        // Convert ROS2 msg to PCL cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // Remove NaNs
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        // Estimate normals
        pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

        //pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        ne.setNumberOfThreads(std::thread::hardware_concurrency());

        ne.setSearchMethod(tree);
        ne.setInputCloud(cloud);
        ne.setKSearch(30);
        ne.compute(*normals);

        // Region growing segmentation
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize(50);
        reg.setMaxClusterSize(100000);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(30);
        reg.setInputCloud(cloud);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);  // radians
        reg.setCurvatureThreshold(1.0);

        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);
        if (clusters.empty()) {
            RCLCPP_WARN(this->get_logger(), "No clusters found.");
            return;
        }

        // Build KD-tree for nearest point search
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);
        pcl::PointXYZ search_point;
        search_point.x = last_click_.point.x;
        search_point.y = last_click_.point.y;
        search_point.z = last_click_.point.z;

        // Retrieve all neighbors sorted by distance
        std::vector<int> nn_indices(cloud->points.size());
        std::vector<float> nn_dists(cloud->points.size());
        int found = kdtree.nearestKSearch(search_point, cloud->points.size(), nn_indices, nn_dists);
        if (found <= 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to find nearest points in cloud.");
            return;
        }

        // Iterate through neighbors until we find one belonging to a cluster
        int highlight_cluster = -1;
        int highlight_point_idx = -1;
        for (int idx : nn_indices) {
            for (size_t c = 0; c < clusters.size(); ++c) {
                const auto& pts = clusters[c].indices;
                if (std::find(pts.begin(), pts.end(), idx) != pts.end()) {
                    highlight_cluster = c;
                    highlight_point_idx = idx;
                    break;
                }
            }
            if (highlight_cluster >= 0) break;
        }

        if (highlight_cluster < 0) {
            RCLCPP_WARN(this->get_logger(), "No clicked point cluster found among nearest neighbors.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Highlighting cluster %d containing point index %d", highlight_cluster, highlight_point_idx);

        // Check completion and log result
        bool completed = isClusterCompleted(cloud, clusters[highlight_cluster]);
        if (completed) {
            RCLCPP_INFO(this->get_logger(), "Cluster %d is completed.", highlight_cluster);
        } else {
            RCLCPP_INFO(this->get_logger(), "Cluster %d is not completed.", highlight_cluster);
        }

        // Publish only highlighted cluster in red
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (int idx : clusters[highlight_cluster].indices) {
            pcl::PointXYZRGB pt;
            pt.x = cloud->points[idx].x;
            pt.y = cloud->points[idx].y;
            pt.z = cloud->points[idx].z;
            pt.r = 255;
            pt.g = 0;
            pt.b = 0;
            colored_cloud->points.push_back(pt);
        }

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*colored_cloud, output_msg);
        output_msg.header = msg->header;
        cloud_pub_->publish(output_msg);

        publish_endpoints(cluster_info);
        publish_cluster_closed(completed);

        // has_click_ = false; // only process once for each clicked point
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr endpoint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr normal_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cluster_closed_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr unclosed_pt_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr closed_pt_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr next_wall_pub_;

    geometry_msgs::msg::PointStamped last_click_;
    bool has_click_;

    Endpoints cluster_info;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RegionGrowingSegmentationNode>());
    rclcpp::shutdown();
    return 0;
}