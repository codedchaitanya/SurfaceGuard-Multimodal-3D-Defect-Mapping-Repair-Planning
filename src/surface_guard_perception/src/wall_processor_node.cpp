#include "surface_guard_perception/wall_processor.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/common/common.h"

using std::placeholders::_1;

WallProcessorNode::WallProcessorNode() : Node("wall_processor_node") {
    // Load Params
    this->declare_parameter("leaf_size", 0.02);
    this->declare_parameter("ransac_dist_thresh", 0.02);
    this->declare_parameter("cluster_tolerance", 0.05);
    this->declare_parameter("min_cluster_size", 20);
    this->declare_parameter("max_cluster_size", 5000);

    leaf_size_ = this->get_parameter("leaf_size").as_double();
    ransac_dist_thresh_ = this->get_parameter("ransac_dist_thresh").as_double();
    cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
    min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
    max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();

    // Setup Pub/Sub
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", rclcpp::SensorDataQoS(), 
        std::bind(&WallProcessorNode::topic_callback, this, _1));

    wall_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/wall_plane", 1);
    defect_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/defect_cloud", 1);
    defect_data_pub_ = this->create_publisher<surface_guard_interfaces::msg::DefectArray>("/perception/defects", 1);

    RCLCPP_INFO(this->get_logger(), "Wall Processor Node Started.");
}

void WallProcessorNode::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *raw_cloud);

    if (raw_cloud->empty()) return;

    // 1. Voxel Grid Downsampling
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(raw_cloud);
    sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    sor.filter(*downsampled_cloud);

    if (downsampled_cloud->size() < 100) return; // Safety check

    // 2. RANSAC Plane Segmentation
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ransac_dist_thresh_);
    seg.setInputCloud(downsampled_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) return;

    // 3. Extract Wall and Defects
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr wall_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr defect_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    
    extract.setInputCloud(downsampled_cloud);
    extract.setIndices(inliers);
    
    // Get Wall
    extract.setNegative(false);
    extract.filter(*wall_cloud);
    
    // Get Defects (Outliers)
    extract.setNegative(true);
    extract.filter(*defect_cloud);

    // Publish Visualization Clouds
    sensor_msgs::msg::PointCloud2 wall_msg, defect_msg;
    pcl::toROSMsg(*wall_cloud, wall_msg);
    pcl::toROSMsg(*defect_cloud, defect_msg);
    wall_msg.header = msg->header;
    defect_msg.header = msg->header;
    wall_pub_->publish(wall_msg);
    defect_cloud_pub_->publish(defect_msg);

    // 4. Cluster Defects and Publish Data
    if (!defect_cloud->empty()) {
        process_defects(defect_cloud, msg->header.frame_id);
    }
}

void WallProcessorNode::process_defects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string frame_id) {
    // Euclidean Clustering: Group nearby outlier points into single "defect" objects
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(cluster_tolerance_); 
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    surface_guard_interfaces::msg::DefectArray defect_array_msg;
    defect_array_msg.header.frame_id = frame_id;
    defect_array_msg.header.stamp = this->now();

    int id_counter = 0;
    for (const auto& indices : cluster_indices) {
        // Calculate Centroid of the cluster
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, indices, centroid);

        surface_guard_interfaces::msg::Defect defect;
        defect.defect_type = "surface_anomaly"; // In pure geometry, we only know it's an anomaly
        defect.centroid.x = centroid[0];
        defect.centroid.y = centroid[1];
        defect.centroid.z = centroid[2];
        defect.surface_area = indices.indices.size() * leaf_size_ * leaf_size_; // Approx area
        defect.confidence = 1.0; // It physically exists

        defect_array_msg.defects.push_back(defect);
    }

    defect_data_pub_->publish(defect_array_msg);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallProcessorNode>());
    rclcpp::shutdown();
    return 0;
}