#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/segmentation/region_growing.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>  // <-- This is required

class RegionGrowingSegmentationNode : public rclcpp::Node
{
public:
    RegionGrowingSegmentationNode()
    : Node("region_growing_segmentation_node")
    {
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/zed/zed_node/mapping/fused_cloud", 10,
            std::bind(&RegionGrowingSegmentationNode::cloud_callback, this, std::placeholders::_1));

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/clustered_cloud_rgb", 10);
        RCLCPP_INFO(this->get_logger(), "Region Growing Segmentation Node started.");
    }

private:
    bool is_normal_parallel_to_z(const pcl::PointIndices& indices,
        const pcl::PointCloud<pcl::Normal>::Ptr& normals,
        double angle_threshold_deg = 20.0)
    {
        Eigen::Vector3f avg_normal(0, 0, 0);
        for (int idx : indices.indices)
        {
        const auto& n = normals->points[idx];
        avg_normal += Eigen::Vector3f(n.normal_x, n.normal_y, n.normal_z);
        }
        avg_normal.normalize();

        double dot = std::abs(avg_normal.dot(Eigen::Vector3f(0, 0, 1))); // cos(theta)
        double cos_thresh = std::cos(angle_threshold_deg * M_PI / 180.0);
        return dot > cos_thresh;
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) return;

        // Step 0: Remove NaNs
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
        if (cloud->empty()) return;

        // Random downsampling
        indices.resize(cloud->points.size());
                std::iota(indices.begin(), indices.end(), 0);
        std::random_shuffle(indices.begin(), indices.end());

        size_t sample_size = static_cast<size_t>(cloud->points.size() * 0.80);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
        cloud_downsampled->points.reserve(sample_size);

        for (size_t i = 0; i < sample_size; ++i)
        {
            cloud_downsampled->points.push_back(cloud->points[indices[i]]);
        }
        cloud_downsampled->width = cloud_downsampled->points.size();
        cloud_downsampled->height = 1;
        cloud_downsampled->is_dense = true;

        if (cloud_downsampled->empty())
            return;

        // Estimate normals
        pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

        //pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        ne.setNumberOfThreads(std::thread::hardware_concurrency());

        ne.setSearchMethod(tree);
        ne.setInputCloud(cloud_downsampled);
        ne.setKSearch(30);
        ne.compute(*normals);

        // Region growing segmentation
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize(50);
        reg.setMaxClusterSize(100000);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(30);
        reg.setInputCloud(cloud_downsampled);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);  // radians
        reg.setCurvatureThreshold(1.0);

        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);

        // Create output colored cloud manually
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        std::vector<uint8_t> color_lut = {
            255, 0, 0,   // red
            0, 255, 0,   // green
            0, 0, 255,   // blue
            255, 255, 0, // yellow
            0, 255, 255, // cyan
            255, 0, 255, // magenta
            255, 127, 0, // orange
            127, 0, 255, // violet
            0, 127, 255, // sky
            127, 255, 0  // lime
        };
        size_t color_count = color_lut.size() / 3;

        for (size_t i = 0; i < clusters.size(); ++i)
        {
            const auto& indices = clusters[i];

            // Skip vertical clusters
            if (is_normal_parallel_to_z(indices, normals, 20.0))
                continue;

            uint8_t r = color_lut[(i % color_count) * 3 + 0];
            uint8_t g = color_lut[(i % color_count) * 3 + 1];
            uint8_t b = color_lut[(i % color_count) * 3 + 2];

            for (int idx : indices.indices)
            {
                const auto& pt = cloud_downsampled->points[idx];
                pcl::PointXYZRGB pt_rgb;
                pt_rgb.x = pt.x;
                pt_rgb.y = pt.y;
                pt_rgb.z = pt.z;
                pt_rgb.r = r;
                pt_rgb.g = g;
                pt_rgb.b = b;
                colored_cloud->points.push_back(pt_rgb);
            }
        }

        colored_cloud->width = colored_cloud->points.size();
        colored_cloud->height = 1;
        colored_cloud->is_dense = true;

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*colored_cloud, output_msg);
        output_msg.header = msg->header;
        cloud_pub_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RegionGrowingSegmentationNode>());
    rclcpp::shutdown();
    return 0;
}