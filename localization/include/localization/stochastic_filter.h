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
    StochasticFilter();

    void addPoseToOdometryQueue(const Eigen::Matrix4f &pose);

    void calculateCovarianceGains();

    void setGPSCovarianceMatrix(const Eigen::Matrix3f& gps_covariance);

    float getGPSGain() const;

    float getOdometryGain() const;

private:
    Eigen::Matrix3f calculateCovarianceMatrix(const std::vector<Eigen::Matrix4f> &poses);

std::vector<Eigen::Matrix4f> odometry_poses_;
Eigen::Matrix3f gps_covariance_matrix_;
float gps_gain_{0.0f}, odometry_gain_{1.0f};

const std::size_t covariance_filter_queue_size_{10}, minimum_poses_for_covariance_{10};
int circular_vector_counter_{0};
};
#endif // STOCHASTIC_FILTER_H_
