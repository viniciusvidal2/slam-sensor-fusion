#ifndef LOCALIZATION_NODE_H
#define LOCALIZATION_NODE_H

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
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/ndt.h>
#include <pcl/features/normal_3d.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include "localization/geo_lib.hpp"
#include "localization/global_map_frames_manager.h"
#include "localization/icp_point_to_point.h"
#include "localization/point_cloud_processing.hpp"

using PointT = pcl::PointXYZ;

class LocalizationNode
{
public:
    /// @brief Constructor
    LocalizationNode(ros::NodeHandle nh);

private:
    /// @brief Compute the pose prediction from the odometry message
    /// @param odom_msg The odometry message
    /// @param odom_T_sensor_current The transformation matrix for the sensor in odometry frame
    /// @param map_T_sensor_current_odom The transformation matrix for the sensor in map frame based on odometry
    inline void computePosePredictionFromOdometry(const nav_msgs::Odometry::ConstPtr &odom_msg,
                                                  Eigen::Matrix4f &odom_T_sensor_current,
                                                  Eigen::Matrix4f &map_T_sensor_current_odom) const;

    /// @brief Compute the coarse pose from GPS and compass in the map frame
    /// @param gps_msg The GPS message
    /// @return The transformation matrix from the map to the sensor frame using gps and compass
    const Eigen::Matrix4f computeGpsCoarsePoseInMapFrame(const sensor_msgs::NavSatFix::ConstPtr &gps_msg) const;

    /// @brief Build a nav_msgs::Odometry message from a transformation matrix
    /// @param T The transformation matrix
    /// @param frame_id The frame id
    /// @param child_frame_id The child frame id
    /// @param stamp The timestamp
    /// @return The nav_msgs::Odometry message
    inline nav_msgs::Odometry buildNavOdomMsg(const Eigen::Matrix4f &T,
                                              const std::string &frame_id,
                                              const std::string &child_frame_id,
                                              const ros::Time &stamp) const;

    /// @brief Compute the pose weights from the covariance matrices
    /// @param gps_msg The GPS message
    /// @param odom_msg The odometry message
    /// @param odom_gain The odometry gain
    /// @param gps_gain The GPS gain
    void computePoseGainsFromCovarianceMatrices(const sensor_msgs::NavSatFix::ConstPtr &gps_msg,
                                                const nav_msgs::Odometry::ConstPtr &odom_msg,
                                                float &odom_gain, float &gps_gain) const;

    /// @brief Initialize the poses with the first reading
    /// @param gps_msg The GPS message
    /// @param odom_msg The odometry message
    void initializePosesWithFirstReading(const sensor_msgs::NavSatFix::ConstPtr &gps_msg,
                                         const nav_msgs::Odometry::ConstPtr &odom_msg);

    /// @brief Simple velocity filter to avoid glitches
    /// @param pose The pose to filter
    /// @param previous_pose The previous pose
    /// @param time_diff The time difference between the poses
    void velocityFilter(Eigen::Matrix4f &pose,
                        const Eigen::Matrix4f &previous_pose,
                        const float time_diff) const;

    /// @brief Perform the coarse alignment between the scan and the map
    /// @param scan_cloud The scan point cloud
    /// @param map_cloud The map point cloud
    /// @return True if the alignment was successful, false otherwise
    bool performCoarseAlignment(const pcl::PointCloud<PointT>::Ptr &scan_cloud,
                                const pcl::PointCloud<PointT>::Ptr &map_cloud);

    /// @brief Callback for the localization node
    /// @param pointcloud_msg The incoming point cloud message
    /// @param gps_msg The incoming GPS message
    /// @param odom_msg The incoming odometry message
    void localizationCallback(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg,
                              const sensor_msgs::NavSatFix::ConstPtr &gps_msg,
                              const nav_msgs::Odometry::ConstPtr &odom_msg);

    /// @brief Callback to get the yaw angle from compass
    /// @param msg The compass message
    void compassCallback(const std_msgs::Float64::ConstPtr &msg);

    /// @brief Filter the point cloud to remove the vehicle
    /// @param cloud The point cloud to filter
    void filterVehicleBox(pcl::PointCloud<PointT> &cloud);

    // Synchronizer policy
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::PointCloud2,
        sensor_msgs::NavSatFix,
        nav_msgs::Odometry>
        SyncPolicy;

    // Subscribers and synchronizer
    ros::Subscriber compass_subscription_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    /// @brief Publishers
    ros::Publisher map_T_sensor_pub_;
    ros::Publisher map_T_sensor_prior_pub_;
    ros::Publisher map_T_sensor_odom_pub_;
    ros::Publisher map_T_sensor_gps_pub_;
    ros::Publisher cropped_scan_pub_;
    ros::Publisher map_pub_;

    /// @brief Yaw angle from compass
    float current_compass_yaw_{0.0f}; // -M_PI to M_PI [RAD]

    /// @brief Reference transforms in the node
    Eigen::Matrix4f map_T_sensor_;
    Eigen::Matrix4d map_T_global_;
    Eigen::Matrix4f odom_T_sensor_previous_;
    Eigen::Matrix4f map_T_ref_;
    Eigen::Matrix4f map_T_odom_;

    /// @brief ICP object and parameters
    std::shared_ptr<ICPPointToPoint> icp_;
    int icp_iterations_{16};
    float icp_transform_epsilon_{1e-8f};
    float icp_max_correspondence_dist_{0.05f};
    float icp_mean_accepted_error_{0.05f};

    /// @brief Global frame manager object
    std::shared_ptr<GlobalMapFramesManager> global_map_frames_manager_;

    /// @brief Map clouds parameters
    pcl::PointCloud<PointT>::Ptr map_cloud_;
    pcl::PointCloud<PointT>::Ptr ref_cropped_map_cloud_;
    int max_map_optimization_poses_{50};
    float map_voxel_size_{0.1f};           // [m]
    float ref_frame_distance_{3.0f};       // [m]
    const float cloud_crop_radius_{10.0f}; // [m]
    std::string relative_folder_path_{"/Desktop/map_data"};
    std::string map_name_{"map"};

    /// @brief Flags
    bool debug_{false};
    bool first_time_{true};
    bool coarse_alignment_complete_{false};

    /// @brief Previous odom message stamp received in the callback
    ros::Time previous_odom_stamp_{ros::Time(0)};

    /// @brief Covariance filter parameters
    bool pose_gains_calculation_option_{false};
    float odom_fixed_gain_{0.95};
    float gps_fixed_gain_{0.05};

    /// @brief Velocity filter parameters
    float max_rover_velocity_{1.6f}; // [m/s]

    /// @brief Vehicle extrinsics, vehicle with respect to lidar
    Eigen::Matrix4f lidar_T_vehicle_{Eigen::Matrix4f::Identity()};

    /// @brief Vehicle box filter
    Eigen::Vector3f vehicle_box_size_{}; // [m]
};

#endif
