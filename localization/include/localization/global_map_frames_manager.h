#ifndef GLOBAL_MAP_FRAMES_MANAGER_H
#define GLOBAL_MAP_FRAMES_MANAGER_H
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <signal.h>
#include <dirent.h>
#include <unistd.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include "localization/geo_lib.hpp"

using PointT = pcl::PointXYZ;

class GlobalMapFramesManager
{
public:
    /// @brief Constructor
    /// @param data_folder The folder where the map data is stored
    /// @param map_name The name of the map
    /// @param num_poses_max The maximum number of poses to consider for the global to map transform computation
    GlobalMapFramesManager(const std::string data_folder, const std::string map_name, const std::size_t num_poses_max);

    /// @brief Get the map cloud
    /// @param voxel_size The voxel size to downsample the map cloud
    /// @return The map cloud
    pcl::PointCloud<PointT>::Ptr getMapCloud(const float voxel_size) const;

    /// @brief Get the transformation from the global to the map frame
    /// @return The transformation matrix from the global to the map frame
    Eigen::Matrix4d getMapTGlobal();

    /// @brief Get the closest altitude in the lat lon alt table
    /// @param lat The latitude
    /// @param lon The longitude
    /// @return The closest altitude
    float getClosestAltitude(const double lat, const double lon) const;

private:
    /// @brief Load the odometry positions from the file
    /// @param odom_positions_file The file with the odometry positions
    /// @return A vector with the odometry positions
    std::vector<Eigen::Vector3d> loadOdometryPositions(const std::string &odom_positions_file) const;

    /// @brief Load the global info from the GPS and IMU
    /// @param gps_rpy_positions_file The file with the GPS and IMU positions
    /// @return A vector with the latitude, longitude, altitude and yaw
    std::vector<std::pair<Eigen::Vector3d, float>> loadGlobalInfo(const std::string &gps_rpy_positions_file);

    /// @brief Compute the transformation from the global to the map frame
    /// @param latlonalt The latitude, longitude and altitude vector in global frame
    /// @param compass_yaw The yaw angles from the compass
    /// @return The transformation matrix from the global to the map frame
    Eigen::Matrix4d computeMapTGlobal(const std::vector<Eigen::Vector3d> &latlonalt,
                                      const std::vector<float> &compass_yaw) const;

    /// @brief Load the map clouds, save them to disk and return the merged cloud
    /// @param voxel_size The voxel size to downsample the merged cloud
    /// @return The merged cloud
    pcl::PointCloud<PointT>::Ptr mergeScansAndSave(const float voxel_size) const;

    /// @brief Filter the bad readings from the odometry and global info vectors
    /// @param odom_positions The odometry positions
    /// @param latlonalt_yaw The latitude, longitude, altitude and yaw vectors
    /// @return True if the filtering was successful
    bool filterBadReadings(std::vector<Eigen::Vector3d> &odom_positions,
                           std::vector<std::pair<Eigen::Vector3d, float>> &latlonalt_yaw) const;

    /// @brief Data folder where the map data is stored
    std::string data_folder_, map_name_;

    /// @brief Maximum number of poses to consider for the map
    std::size_t num_poses_max_;

    /// @brief The GPS altitude table for reference - avoid altitude drifts from different runs
    std::vector<Eigen::Vector3d> gps_altitude_table_;
};
#endif
