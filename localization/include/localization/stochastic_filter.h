#ifndef STOCHASTIC_FILTER_H_
#define STOCHASTIC_FILTER_H_
#include <vector>
#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

class StochasticFilter
{
public:
    /// @brief Constructor
    StochasticFilter();

    /// @brief Add a pose to the odometry queue
    void addPoseToOdometryQueue(const Eigen::Matrix4f &pose);

    /// @brief Calculate the covariance matrix and gains based on the GPS and Odometry covariances 
    void calculateCovarianceGains();

    /// @brief Sets the GPS covariance matrix to compute gains
    /// @param gps_covariance 
    void setGPSCovarianceMatrix(const Eigen::Matrix3f& gps_covariance);

    /// @brief Get the GPS gain
    float getGPSGain() const;

    /// @brief Get the odometry gain
    float getOdometryGain() const;

private:
    /// @brief Calculates XYZ covariance matrix from poses
    /// @param poses: input poses queue
    /// @return A 3x3 covariance matrix
    Eigen::Matrix3f calculateCovarianceMatrix(const std::vector<Eigen::Matrix4f> &poses) const;

    /// @brief Odometry poses queue to compute oodmetry covariance
    std::vector<Eigen::Matrix4f> odometry_poses_;

    /// @brief GPS covariance matrix set from sensor
    Eigen::Matrix3f gps_covariance_matrix_;

    /// @brief Gains based on covariance
    float gps_gain_{0.0f};
    float odometry_gain_{1.0f};

    /// @brief Covariance queue parameters
    const std::size_t covariance_filter_queue_size_{10};
    const std::size_t minimum_poses_for_covariance_{10};
    int circular_vector_counter_{0};
};
#endif // STOCHASTIC_FILTER_H_
