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

    void addPosesToQueues(const Eigen::Matrix4f &gps_pose, const Eigen::Matrix4f &odometry_pose);

    void calculateCovarianceGains(float& gps_pose_gain, float& odometry_pose_gain);

private:
    Eigen::Matrix3f calculateCovarianceMatrix(const std::vector<Eigen::Matrix4f> &poses);

std::vector<Eigen::Matrix4f> gps_poses_, odometry_poses_;
const std::size_t covariance_filter_queue_size_{10}, minimum_poses_for_covariance_{10};
int circular_vector_counter_{0};
};
#endif // STOCHASTIC_FILTER_H_
