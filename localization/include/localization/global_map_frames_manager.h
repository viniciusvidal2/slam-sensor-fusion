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
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include "localization/geo_lib.h"

using PointT = pcl::PointXYZ;

class GlobalMapFramesManager
{
public:
    GlobalMapFramesManager(const std::string data_folder, const std::string map_name, const std::size_t num_poses_max);

    pcl::PointCloud<PointT>::Ptr getMapCloud(const float voxel_size) const;

    Eigen::Matrix4d getMapTGlobal() const;

private:
    std::vector<Eigen::Vector3d> loadOdometryPositions(const std::string &odom_positions_file) const;

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> loadGlobalInfoAndRpy(const std::string &gps_rpy_positions_file) const;

    Eigen::Matrix4d computeMapTGlobal(const std::vector<Eigen::Vector3d> &odom_positions,
                                      const std::vector<Eigen::Vector3d> &latlonalt,
                                      const std::vector<Eigen::Vector3d> &compass_rpy) const;

    pcl::PointCloud<PointT>::Ptr mergeScansAndSave(const float voxel_size) const;

    std::string data_folder_, map_name_;
    std::size_t num_poses_max_;
};
#endif
