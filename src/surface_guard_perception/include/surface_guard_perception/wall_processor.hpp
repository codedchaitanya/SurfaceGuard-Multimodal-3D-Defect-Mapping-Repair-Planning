#ifndef WALL_PROCESSOR_HPP_
#define WALL_PROCESSOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "surface_guard_interfaces/msg/defect.hpp"
#include "surface_guard_interfaces/msg/defect_array.hpp"

// PCL Includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class WallProcessorNode : public rclcpp::Node {
public:
    WallProcessorNode();

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    // Helper to cluster outliers into distinct defect objects
    void process_defects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string frame_id);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr wall_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr defect_cloud_pub_;
    rclcpp::Publisher<surface_guard_interfaces::msg::DefectArray>::SharedPtr defect_data_pub_;

    // Parameters
    double leaf_size_;
    double ransac_dist_thresh_;
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
};

#endif // WALL_PROCESSOR_HPP_