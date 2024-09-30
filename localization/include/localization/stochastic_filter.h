#ifndef STOCHASTIC_FILTER_H_
#define STOCHASTIC_FILTER_H_
#include <vector>
#include <iostream>
#include <iomanip>
#include <numeric>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

class StochasticFilter
{
public:
    /// @brief Constructor
    StochasticFilter(const std::size_t queue_size = 10, const float n_std_dev_threshold = 1.0f);

    /// @brief Add a pose to the odometry queue
    /// @param origin_pose_current The current pose to add to the queue
    void addPoseToQueue(const Eigen::Matrix4f &origin_pose_current);

    /// @brief Set the queue size
    /// @param queue_size The queue size
    inline void setQueueSize(const std::size_t queue_size);

    /// @brief Set the probability threshold
    /// @param n_std_dev_threshold The probability threshold
    inline void setProbabilityThreshold(const float n_std_dev_threshold);

    /// @brief Set the maximum scan traveled distance based on the maximum velocity
    /// @param max_linear_velocity The maximum linear velocity
    void setMaximumLinearVelocity(const float max_linear_velocity);

    /// @brief Apply a Gaussian filter to the current pose
    /// @param origin_pose_previous The previous pose
    /// @param origin_pose_current The current pose
    /// @return The filtered pose
    Eigen::Matrix4f applyGaussianFilterToCurrentPose(const Eigen::Matrix4f &origin_pose_previous, const Eigen::Matrix4f &origin_pose_current) const;

private:
    /// @brief Compute the pose z score of current pose based on the previous pose and transition queue
    /// @param origin_pose_previous The previous pose
    /// @param origin_pose_current The current pose
    /// @return The probability of the current pose
    float computePoseZScore(const Eigen::Matrix4f &origin_pose_previous, const Eigen::Matrix4f &origin_pose_current) const;

    /// @brief Compute the normal probability density function
    /// @param x The value to evaluate
    /// @param mean The mean of the distribution
    /// @param stddev The standard deviation of the distribution
    /// @return The probability density function value
    inline float normalPDF(const float x, const float mean, const float stddev) const;

    /// @brief Compute the normal cumulative distribution function
    /// @param x The value to evaluate
    /// @param mean The mean of the distribution
    /// @param stddev The standard deviation of the distribution
    /// @return The cumulative distribution function value
    inline float normalCDF(double x, double mean, double stddev) const;

    /// @brief Pose transition queue and previous pose
    Eigen::Matrix4f origin_pose_previous_;
    std::vector<Eigen::Matrix4f> pose_transition_queue_;

    /// @brief Weights vector to perform a weighted average
    std::vector<float> weights_;
    
    /// @brief Queue size
    std::size_t queue_size_;

    /// @brief Probability threshold
    float n_std_dev_threshold_;

    /// @brief Maximum max distance that can be traveled in one scan
    float max_distance_per_scan_;
};
#endif // STOCHASTIC_FILTER_H_
