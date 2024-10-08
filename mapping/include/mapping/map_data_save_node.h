#ifndef MAP_DATA_SAVE_NODE_H
#define MAP_DATA_SAVE_NODE_H
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
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "mapping/file_manipulation.hpp"

using PointT = pcl::PointXYZ;

class MapDataSaver
{
public:
    /// @brief Constructor
    MapDataSaver(ros::NodeHandle& nh);

private:
    /// @brief The mapping callback
    /// @param pointcloud_msg The point cloud message
    /// @param gps_msg The GPS message
    /// @param odom_msg The odometry message
    void mappingCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg,
                  const sensor_msgs::NavSatFix::ConstPtr& gps_msg,
                  const nav_msgs::Odometry::ConstPtr& odom_msg);

    /// @brief Callback to run at node kill
    void onShutdown();

    /// @brief The synchronization policy
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::PointCloud2,
        sensor_msgs::NavSatFix,
        nav_msgs::Odometry>;

    /// @brief Subscribers and synchronizer
    ros::Subscriber compass_subscription_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;  
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    /// @brief Paths
    std::string folder_save_path_;
    std::string odometry_file_path_;
    std::string gps_imu_poses_file_path_;

    ///@brief Point cloud to save the map in tiles
    int cloud_counter_{0};
    constexpr static int cloud_save_rate_{10};
    pcl::PointCloud<PointT>::Ptr cloud_map_frame_;

    /// @brief Yaw angle from compass
    double current_compass_yaw_{0.0}; // -M_PI to M_PI [RAD]

    /// @brief Debug flag
    bool debug_{false};
};

#endif  // MAP_DATA_SAVE_NODE_H
