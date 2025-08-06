#include <memory>
#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/random_sample.h>

#include <chrono>

class RegionGrowingSegmentationNode : public rclcpp::Node
{
public:
    RegionGrowingSegmentationNode()
    : Node("region_growing_segmentation")
    {
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/os0_cloud_node/points", 10,
            std::bind(&RegionGrowingSegmentationNode::cloudCallback, this, std::placeholders::_1));

        clusters_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "clustered_cloud", 10);

        RCLCPP_INFO(this->get_logger(), "RegionGrowingSegmentationNode initialized");
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        //auto start = std::chrono::high_resolution_clock::now();

        // Record processing timestamp
        rclcpp::Time processing_time = msg->header.stamp;

        // Convert to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty cloud, skipping");
            return;
        }

        // Remove NaNs
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Cloud after NaN removal is empty, skipping");
            return;
        }

        // Random downsampling (20%)
        // auto cloud_downsampled = random_downsample(cloud, 0.2);
        float downsample_rate_ = 0.2;
        std::size_t sample_size = static_cast<std::size_t>(cloud->size() * downsample_rate_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
        {
        pcl::RandomSample<pcl::PointXYZ> sampler;
        sampler.setInputCloud(cloud);
        sampler.setSample(sample_size);
        sampler.filter(*cloud_downsampled);
        }

        if (cloud_downsampled->empty())
            return;

        // Filter out points behind base_link
        cloud_downsampled = filter_points_in_front(cloud_downsampled);

        // Estimate normals
        pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        ne.setNumberOfThreads(std::thread::hardware_concurrency());

        ne.setSearchMethod(tree);
        ne.setInputCloud(cloud_downsampled);
        ne.setKSearch(20); //use the k nearest neighbors of each point to estimate its normal vector.
        ne.compute(*normals);

        /* //TODO: the following method is not working, possible cause: the normals of points are not ideal
        // But after filtering the point, still need to compute the normal to reflect the structure of filtered point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr normals_filtered(new pcl::PointCloud<pcl::Normal>);

        const Eigen::Vector3f z_axis(0.0f, 0.0f, 1.0f);
        const float dot_threshold = std::cos(80.0 * M_PI / 180.0);  // Allow 10° deviation from Z-axis

        for (size_t i = 0; i < normals->size(); ++i)
        {
            Eigen::Vector3f normal(normals->points[i].normal_x,
                                normals->points[i].normal_y,
                                normals->points[i].normal_z);

            if (normal.norm() == 0) continue; // Skip invalid normals

            float dot = std::fabs(normal.normalized().dot(z_axis));
            if (dot < dot_threshold)  // Keep normals NOT parallel to Z
            {
                cloud_filtered->points.push_back(cloud_downsampled->points[i]);
                normals_filtered->points.push_back(normals->points[i]);
            }
        }
        cloud_filtered->width = cloud_filtered->points.size();
        cloud_filtered->height = 1;
        cloud_filtered->is_dense = false;

        normals_filtered->width = normals_filtered->points.size();
        normals_filtered->height = 1;
        normals_filtered->is_dense = false;
        */

        // Region growing segmentation
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize(50);
        reg.setMaxClusterSize(100000);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(20); 

        reg.setInputCloud(cloud_downsampled);
        reg.setInputNormals(normals);
        //reg.setInputCloud(cloud_filtered);
        //reg.setInputNormals(normals_filtered);

        reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);  // radians
        reg.setCurvatureThreshold(1.0);

        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);

        // Filter out clusters whose normals are nearly vertical
        std::vector<pcl::PointIndices> filtered_clusters;
        for (const auto& cluster : clusters) {
            if (!isNormalParallelToZ(cluster, normals, 20.0)) {
                filtered_clusters.push_back(cluster);
            }
        }

        // Publish clustered points with labels and timestamp
        publishClusters(cloud_downsampled, filtered_clusters, processing_time, msg->header.frame_id);

        /*
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end - start;

        RCLCPP_INFO(this->get_logger(), "cloudCallback took %.3f ms", elapsed.count());
        */
    }

    // Check if average normal of a cluster is within angle_threshold_deg of the Z axis
    bool isNormalParallelToZ(
        const pcl::PointIndices& indices,
        const pcl::PointCloud<pcl::Normal>::Ptr& normals,
        double angle_threshold_deg = 20.0)
    {
        Eigen::Vector3f avg_normal(0.0f, 0.0f, 0.0f);
        for (int idx : indices.indices)
        {
            const auto& n = normals->points[idx];
            avg_normal += Eigen::Vector3f(n.normal_x, n.normal_y, n.normal_z);
        }
        avg_normal.normalize();
        float angle_rad = std::acos(std::abs(avg_normal.dot(Eigen::Vector3f(0.0f, 0.0f, 1.0f))));
        float angle_deg = angle_rad * 180.0f / M_PI;
        return angle_deg < angle_threshold_deg;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr random_downsample(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, double rate)
    {
        size_t N = input->points.size();
        size_t sample_size = static_cast<size_t>(N * rate);

        std::vector<int> indices(N);
        std::iota(indices.begin(), indices.end(), 0);
        std::random_shuffle(indices.begin(), indices.end());

        pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>());
        out->points.reserve(sample_size);
        for (size_t i = 0; i < sample_size; ++i)
            out->points.push_back(input->points[indices[i]]);

        out->width = out->points.size();
        out->height = 1;
        out->is_dense = true;
        return out;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_points_in_front(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
    {
        Eigen::Matrix4f T_lidar_to_base;
        T_lidar_to_base << 
            0.7071,  0.7071,  0.0,  0.0,
            -0.7071,  0.7071,  0.0, 0.0,
            0.0,     0.0,     1.0,  0.0,
            0.0,     0.0,     0.0,  1.0;

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& pt : input->points)
        {
            Eigen::Vector4f p_lidar(pt.x, pt.y, pt.z, 1.0f);
            Eigen::Vector4f p_base = T_lidar_to_base * p_lidar;
            if (p_base.x() >= 0.0f)
                filtered->points.push_back(pt);
        }

        filtered->width = filtered->points.size();
        filtered->height = 1;
        filtered->is_dense = true;
        return filtered;
    }

    void publishClusters(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const std::vector<pcl::PointIndices> &clusters,
        const rclcpp::Time &timestamp,
        const std::string &frame_id)
    {
        pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud(new pcl::PointCloud<pcl::PointXYZL>());
        labeled_cloud->points.reserve(cloud->points.size());
        for (size_t cid = 0; cid < clusters.size(); ++cid) {
            for (int idx : clusters[cid].indices) {
                pcl::PointXYZL pt;
                pt.x = cloud->points[idx].x;
                pt.y = cloud->points[idx].y;
                pt.z = cloud->points[idx].z;
                pt.label = static_cast<uint32_t>(cid);
                labeled_cloud->points.push_back(pt);
            }
        }
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*labeled_cloud, output_msg);
        output_msg.header.stamp = timestamp;
        output_msg.header.frame_id = frame_id;
        clusters_pub_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RegionGrowingSegmentationNode>());
    rclcpp::shutdown();
    return 0;
}