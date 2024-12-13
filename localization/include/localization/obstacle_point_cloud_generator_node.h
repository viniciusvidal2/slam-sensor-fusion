#ifndef OBSTACLE_POINT_CLOUD_GENERATOR_NODE_H
#define OBSTACLE_POINT_CLOUD_GENERATOR_NODE_H

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <signal.h>
#include <cstdlib>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include "localization/global_map_frames_manager.h"
#include "localization/point_cloud_processing.hpp"

using PointT = pcl::PointXYZ;

class ObstaclePointCloudGeneratorNode
{
public:
    /// @brief Constructor
    ObstaclePointCloudGeneratorNode(ros::NodeHandle nh);

private:
    /// @brief Callback for the localization node
    /// @param scan_msg The current scan message
    /// @param pose_msg The localized pose message
    void scanCallback(const sensor_msgs::PointCloud2::ConstPtr &scan_msg,
                      const nav_msgs::Odometry::ConstPtr &pose_msg);

    // Synchronizer policy
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::PointCloud2,
        nav_msgs::Odometry>
        SyncPolicy;

    // Subscribers and synchronizer
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    /// @brief Publishers
    ros::Publisher obstacles_lidar_frame_point_cloud_pub_;
    ros::Publisher obstacles_map_frame_point_cloud_pub_;
    ros::Publisher map_point_cloud_pub_;

    /// @brief Global frame manager object
    std::shared_ptr<GlobalMapFramesManager> global_map_frames_manager_;

    /// @brief Map clouds parameters
    pcl::PointCloud<PointT>::Ptr map_cloud_;
    float map_voxel_size_{0.1f}; // [m]
    std::string relative_folder_path_{"/Desktop/map_data"};
    std::string map_name_{"map"};

    /// @brief Scan crop radius
    float scan_crop_radius_{15.0f}; // [m]

    /// @brief KDtree with the map
    pcl::KdTreeFLANN<PointT>::Ptr map_kdtree_;

    /// @brief Flags
    bool debug_{false};
};

#endif
