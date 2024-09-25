#ifndef ICP_POINT_TO_POINT_H
#define ICP_POINT_TO_POINT_H
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
#include <pcl/kdtree/kdtree_flann.h>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

using PointT = pcl::PointXYZ;

class ICPPointToPoint
{
public:
    ICPPointToPoint(const float max_correspondence_dist, const int num_iterations, const float acceptable_mean_error, const float transformation_epsilon);

    void setMaxCorrespondenceDist(const float max_correspondence_dist);

    void setNumIterations(const int num_iterations);

    void setTransformationEpsilon(const float transformation_epsilon);

    void setAcceptableMeanError(const float acceptable_error);

    void setInitialTransformation(const Eigen::Matrix4f &initial_transformation);

    void setInputPointClouds(const pcl::PointCloud<PointT>::Ptr &source_cloud,
                             const pcl::PointCloud<PointT>::Ptr &target_cloud);

    void setDebugMode(bool debug_mode);

    Eigen::Matrix4f calculateAlignmentTransformation();

private:
    Eigen::MatrixX3f convertPclToEigen(const pcl::PointCloud<PointT>::Ptr &cloud);

    void applyTransformation(const Eigen::Matrix4f &transformation, Eigen::MatrixX3f &cloud);

    void filterValidCorrespondencesInClouds(Eigen::MatrixX3f &source_cloud, Eigen::MatrixX3f &target_cloud);

    Eigen::MatrixX3f findSourceCorrespondencesInTargetCloud(const Eigen::MatrixX3f& source_cloud);

    Eigen::Matrix4f calculateStepBestTransformation(const Eigen::MatrixX3f &source_cloud, const Eigen::MatrixX3f &target_cloud);

    float calculateErrorMetric(const Eigen::MatrixX3f &source_cloud, const Eigen::MatrixX3f &target_cloud);

    void printStepDebug(const int i, const float error) const;

    pcl::PointCloud<PointT>::Ptr source_cloud_pcl_;
    pcl::PointCloud<PointT>::Ptr target_cloud_pcl_;
    Eigen::MatrixX3f source_cloud_eigen_;

    float max_correspondence_dist_;
    int num_iterations_;
    float transformation_epsilon_;
    float acceptable_mean_error_;
    float last_error_;
    Eigen::Matrix4f initial_transform_;
    bool debug_mode_;
};

#endif // ICP_POINT_TO_POINT_H
