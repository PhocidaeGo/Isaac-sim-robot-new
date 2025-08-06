#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>

struct ClusterInfo {
    int id;
    Eigen::Vector3f normal;
    bool completed;
    std::vector<int> not_closed_endpoints;
};

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

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/clustered_cloud_rgb", 10);
        cluster_info_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/cluster_info", 10);

        RCLCPP_INFO(this->get_logger(), "Region Growing Segmentation Node started.");
    }

private:
    void click_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        last_click_ = *msg;
        has_click_ = true;
        RCLCPP_INFO(get_logger(),
            "Received clicked point: [%.2f, %.2f, %.2f]",
            last_click_.point.x, last_click_.point.y, last_click_.point.z);
    }

    struct Endpoints { Eigen::Vector3f e1, e2, normal; };

    Endpoints computeEndpointsAndNormal(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const pcl::PointIndices& indices)
    {
        std::vector<Eigen::Vector2f> pts2d;
        pts2d.reserve(indices.indices.size());
        float sum_z = 0.0f;
        for (int idx : indices.indices) {
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
        Eigen::Vector3f e1 = center + dir2d.x()*min_proj*Eigen::Vector3f::UnitX()
                             + dir2d.y()*min_proj*Eigen::Vector3f::UnitY();
        Eigen::Vector3f e2 = center + dir2d.x()*max_proj*Eigen::Vector3f::UnitX()
                             + dir2d.y()*max_proj*Eigen::Vector3f::UnitY();
        Eigen::Vector3f normal(dir2d.y(), -dir2d.x(), 0.0f);
        normal.normalize();
        return {e1, e2, normal};
    }

    bool evaluateCluster(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        pcl::PointIndices& indices,
        ClusterInfo &info)
    {
        const float radius = 1.5f, min_z = 1.0f, max_z = 1.5f;
        const float cos_close = std::cos(60*M_PI/180), cos_exp = std::cos(20*M_PI/180);
        const float thresh = 0.6f;

        std::sort(indices.indices.begin(), indices.indices.end());
        info.not_closed_endpoints.clear();

        // endpoints loop
        for (int eid = 0; eid < 2; ++eid) {
            while (true) {
                auto ep = computeEndpointsAndNormal(cloud, indices);
                info.normal = ep.normal; // store current normal
                Eigen::Vector3f pt = (eid == 0 ? ep.e1 : ep.e2);

                std::vector<int> ext_pts;
                int cls_cnt = 0;
                for (size_t i = 0; i < cloud->points.size(); ++i) {
                    auto &p = cloud->points[i];
                    float dx = p.x - pt.x(), dy = p.y - pt.y();
                    float dxy = std::hypot(dx, dy), dz = p.z - pt.z();
                    if (dxy <= radius && dz >= min_z && dz <= max_z) {
                        if (std::binary_search(indices.indices.begin(), indices.indices.end(), (int)i))
                            cls_cnt++;
                        else ext_pts.push_back(i);
                    }
                }
                if (ext_pts.size() <= thresh*cls_cnt || ext_pts.size() < 3) {
                    info.not_closed_endpoints.push_back(eid+1);
                    break;
                }
                auto ext_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
                for (int i : ext_pts) ext_cloud->points.push_back(cloud->points[i]);
                pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(
                    new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(ext_cloud));
                pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
                ransac.setDistanceThreshold(0.02);
                if (!ransac.computeModel()) {
                    info.not_closed_endpoints.push_back(eid+1);
                    break;
                }
                Eigen::VectorXf coef; ransac.getModelCoefficients(coef);
                Eigen::Vector3f pn(coef[0],coef[1],coef[2]); pn.normalize();
                float dot = std::abs(pn.dot(ep.normal));
                if (dot <= cos_close) {
                    // closed
                    break;
                } else if (dot >= cos_exp) {
                    // expand
                    indices.indices.insert(indices.indices.end(), ext_pts.begin(), ext_pts.end());
                    std::sort(indices.indices.begin(), indices.indices.end());
                    continue;
                } else {
                    info.not_closed_endpoints.push_back(eid+1);
                    break;
                }
            }
        }
        info.completed = info.not_closed_endpoints.empty();
        return info.completed;
    }

    void publishClusterInfos()
    {
        for (auto &info : cluster_infos_) {
            std_msgs::msg::String msg;
            std::stringstream ss;
            ss << "Cluster ID:" << info.id
               << ", normal:[" << info.normal.x() << "," << info.normal.y() << "," << info.normal.z() << "]"
               << ", completed:" << (info.completed ? "true" : "false");
            if (!info.completed) {
                ss << ", not_closed_endpoints:";
                for (int e : info.not_closed_endpoints) ss << e << " ";
            }
            msg.data = ss.str();
            cluster_info_pub_->publish(msg);
        }
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!has_click_) return;
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*msg, *cloud);
        std::vector<int> idx;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, idx);

        auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
        pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::Normal> ne;
        ne.setInputCloud(cloud); ne.setRadiusSearch(0.02); ne.compute(*normals);

        pcl::RegionGrowing<pcl::PointXYZ,pcl::Normal> reg;
        reg.setMinClusterSize(100); reg.setMaxClusterSize(1000000);
        reg.setSearchMethod(std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>());
        reg.setNumberOfNeighbours(30); reg.setInputCloud(cloud);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(3.0f/180.0f*M_PI);
        reg.setCurvatureThreshold(1.0f);

        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);
        if (clusters.empty()) {
            RCLCPP_WARN(get_logger(), "No clusters found."); return;
        }

        cluster_infos_.clear();
        int cid = 1;
        for (auto &ci : clusters) {
            ClusterInfo info;
            info.id = cid++;
            pcl::PointIndices indices = ci; // copy
            evaluateCluster(cloud, indices, info);
            cluster_infos_.push_back(info);
        }
        publishClusterInfos();

        // Highlight logic remains unchanged...
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cluster_info_pub_;
    geometry_msgs::msg::PointStamped last_click_;
    bool has_click_;
    std::vector<ClusterInfo> cluster_infos_;  // global storage for cluster data
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RegionGrowingSegmentationNode>());
    rclcpp::shutdown();
    return 0;
}
