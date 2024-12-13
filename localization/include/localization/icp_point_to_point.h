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
#include <ros/ros.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

using PointT = pcl::PointXYZ;

struct ICPResult
{
    ICPResult(Eigen::Matrix4f T, float error, int iterations, bool has_converged)
        : transformation(T), error(error), iterations(iterations), has_converged(has_converged) {}
    ICPResult(Eigen::Matrix4f T) : transformation(T) {}
    ICPResult() {}

    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    float error{1e6};
    int iterations{0};
    bool has_converged{false};
};

class ICPPointToPoint
{
public:
    /// @brief Constructor
    /// @param max_correspondence_dist The maximum correspondence distance
    /// @param num_iterations The number of iterations
    /// @param acceptable_mean_error The acceptable mean error between correspondences
    /// @param transformation_epsilon The acceptable transformation epsilon
    ICPPointToPoint(const float max_correspondence_dist, const int num_iterations, const float acceptable_mean_error, const float transformation_epsilon);

    /// @brief Set the maximum correspondence distance for the ICP algorithm
    /// @param max_correspondence_dist The maximum correspondence distance
    void setMaxCorrespondenceDist(const float max_correspondence_dist);

    /// @brief Set the number of iterations for the ICP algorithm
    /// @param num_iterations The number of iterations
    void setNumIterations(const int num_iterations);

    /// @brief Set the acceptable transformation epsilon for the ICP algorithm
    /// @param transformation_epsilon The acceptable transformation epsilon
    void setTransformationEpsilon(const float transformation_epsilon);

    /// @brief Set the acceptable mean error for the ICP algorithm
    /// @param acceptable_error The acceptable mean error
    void setAcceptableMeanError(const float acceptable_error);

    /// @brief Set the initial transformation matrix
    /// @param initial_transformation The initial transformation matrix
    void setInitialTransformation(const Eigen::Matrix4f &initial_transformation);

    /// @brief Set the input point clouds for the ICP algorithm
    /// @param source_cloud The source cloud
    void setSourcePointCloud(const pcl::PointCloud<PointT>::Ptr &source_cloud);

    /// @brief Set the target point cloud for the ICP algorithm
    /// @param target_cloud The target cloud
    void setTargetPointCloud(const pcl::PointCloud<PointT>::Ptr &target_cloud);

    /// @brief Set the debug mode flag
    void setDebugMode(bool debug_mode);

    /// @brief Calculate the best transformation to align the source cloud to the target cloud, plus ICP algorithm info
    /// @return The ICP alginment result
    ICPResult calculateAlignment();

private:
    /// @brief Converts the PCL to Eigen format
    /// @param cloud Cloud in PCL format
    /// @return Cloud in Eigen format
    inline Eigen::MatrixX3f convertPclToEigen(const pcl::PointCloud<PointT>::Ptr &cloud) const;

    /// @brief Apply the transformation to the cloud
    /// @param T The transformation matrix
    /// @param cloud The cloud to transform
    inline void applyTransformation(const Eigen::Matrix4f &T, Eigen::MatrixX3f &cloud) const;

    /// @brief Find the correspondences in the target cloud for the source cloud and fill the clouds with them
    /// @param source_cloud The source cloud
    /// @param target_cloud The target cloud
    /// @return True if the correspondences were found, false otherwise
    bool sourceTargetCorrespondences(Eigen::MatrixX3f &source_cloud, Eigen::MatrixX3f &target_cloud) const;

    /// @brief Compute the best transformation from the two clouds
    /// @param source_cloud The source cloud
    /// @param target_cloud The target cloud
    /// @return The best transformation matrix
    Eigen::Matrix4f calculateStepBestTransformation(const Eigen::MatrixX3f &source_cloud, const Eigen::MatrixX3f &target_cloud) const;

    /// @brief Calculate the error metric for the current step
    /// @param source_cloud The source cloud
    /// @param target_cloud The target cloud
    /// @return The error metric value
    inline float calculateErrorMetric(const Eigen::MatrixX3f &source_cloud, const Eigen::MatrixX3f &target_cloud) const;

    /// @brief Print debug information for the current step
    /// @param i The current iteration
    /// @param error The current error
    inline void printStepDebug(const int i, const float error) const;

    /// @brief Point clouds to align and kdtree
    pcl::PointCloud<PointT> target_cloud_pcl_;
    Eigen::MatrixX3f source_cloud_;
    pcl::KdTreeFLANN<PointT> kdtree_;

    /// @brief ICP algorithm parameters
    float max_correspondence_dist_;
    int num_iterations_;
    float transformation_epsilon_;
    float acceptable_mean_error_;
    Eigen::Matrix4f initial_transform_;

    /// @brief ICP last error to compare against
    float last_error_;

    /// @brief Debug mode flag
    bool debug_mode_;
};

#endif // ICP_POINT_TO_POINT_H
