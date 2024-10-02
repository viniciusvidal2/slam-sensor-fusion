#ifndef BRUTE_FORCE_ALIGNMENT_H
#define BRUTE_FORCE_ALIGNMENT_H
#include <vector>
#include <iostream>
#include <iomanip>
#include <numeric>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>

using PointT = pcl::PointXYZ;

class BruteForceAlignment
{
public:
    /// @brief Constructor
    BruteForceAlignment();

    /// @brief Set the step size for the brute force search in the x, y, z directions
    /// @param x_step The step size in the x direction
    /// @param y_step The step size in the y direction
    /// @param z_step The step size in the z direction
    void setXYZStep(const float x_step, const float y_step, const float z_step);

    /// @brief Set the range for the brute force search in the x, y, z directions
    /// @param x The range in the x direction
    /// @param y The range in the y direction
    /// @param z The range in the z direction
    void setXYZRange(const float x, const float y, const float z);

    /// @brief Set the step size for the brute force search in the yaw direction
    /// @param yaw_step The step size in the yaw direction
    void setRotationStep(const float yaw_step);

    /// @brief Set the range for the brute force search in the yaw direction
    /// @param yaw The range in the yaw direction
    void setRotationRange(const float yaw);

    /// @brief Set the mean error threshold for the alignment
    /// @param error_threshold The mean error threshold
    void setMeanErrorThreshold(const float error_threshold);

    /// @brief Set the source cloud for the alignment
    /// @param cloud The source cloud
    void setSourceCloud(const pcl::PointCloud<PointT>::Ptr &cloud);

    /// @brief Set the target cloud for the alignment
    /// @param cloud The target cloud
    void setTargetCloud(const pcl::PointCloud<PointT>::Ptr &cloud);

    /// @brief Set the initial guess for the alignment
    /// @param initial_guess The initial guess for the alignment
    void setInitialGuess(const Eigen::Matrix4f &initial_guess);

    /// @brief Align the source cloud to the target cloud using brute force search
    /// @return True if the alignment was successful, false otherwise
    bool alignClouds();

    /// @brief Check if the first alignment was completed
    /// @return True if the first alignment was completed, false otherwise
    bool firstAlignmentCompleted() const;

    /// @brief Get the best transformation found by the brute force search
    /// @return The best transformation found by the brute force search
    Eigen::Matrix4f getBestTransformation() const;

private:
    /// @brief Create the test sequences for the brute force search
    /// @param x_test The test sequence in the x direction
    /// @param y_test The test sequence in the y direction
    /// @param z_test The test sequence in the z direction
    void createTestTransformSequences(std::vector<float>& x_test, std::vector<float>& y_test, std::vector<float>& z_test, std::vector<float>& yaw_test);

    /// @brief Flag to tell if we have a good first alignment
    bool first_alignment_completed_{false};

    /// @brief Step and range sizes for the brute force search
    float x_step_{0.1f}; // [m]
    float y_step_{0.1f}; // [m]
    float z_step_{0.1f}; // [m]
    float yaw_step_{M_PI/90.0f}; // [RAD]
    float x_range_{0.5f}; // [m]
    float y_range_{0.5f}; // [m]
    float z_range_{0.5f}; // [m]
    float yaw_range_{M_PI/6.0f}; // [RAD]

    /// @brief Source and target clouds for alignment
    pcl::PointCloud<PointT>::Ptr source_cloud_;
    pcl::PointCloud<PointT>::Ptr target_cloud_;

    /// @brief Previous transformation for the brute force search
    Eigen::Matrix4f map_T_sensor_previous_;
    
    /// @brief Best transformation found by the brute force search
    Eigen::Matrix4f map_T_sensor_best_;

    /// @brief Mean error threshold for the alignment
    float alignment_mean_error_threshold_{0.1f}; // [m]
};

#endif // BRUTE_FORCE_ALIGNMENT_H
