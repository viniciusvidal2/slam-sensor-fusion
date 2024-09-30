#include <localization/stochastic_filter.h>

StochasticFilter::StochasticFilter(const std::size_t queue_size, const float n_std_dev_threshold) : 
    queue_size_(queue_size), n_std_dev_threshold_(n_std_dev_threshold)
{
    // Starting the transition queue
    pose_transition_queue_.reserve(queue_size_);
    origin_pose_previous_ = Eigen::Matrix4f::Identity();

    // Weights should be exponential, in decreasing order
    weights_.reserve(queue_size_);
    for (std::size_t i = 0; i < queue_size_; ++i)
    {
        weights_.emplace_back(1); // Temporary simple average
    }
    // Calculate the sum of the weights
    const float weights_sum = std::accumulate(weights_.begin(), weights_.end(), 0.0f);
    // Normalize the weights vector
    for (std::size_t i = 0; i < queue_size_; ++i)
    {
        weights_[i] /= weights_sum;
    }
}

inline void StochasticFilter::setQueueSize(const std::size_t queue_size)
{
    queue_size_ = queue_size;
}

inline void StochasticFilter::setProbabilityThreshold(const float n_std_dev_threshold)
{
    n_std_dev_threshold_ = n_std_dev_threshold;
}

void StochasticFilter::setMaximumLinearVelocity(const float max_linear_velocity)
{
    max_distance_per_scan_ = max_linear_velocity/10.0f; // Scan rate is 10 Hz
}

void StochasticFilter::addPoseToQueue(const Eigen::Matrix4f &origin_pose_current)
{
    if (pose_transition_queue_.size() >= queue_size_)
    {
        pose_transition_queue_.erase(pose_transition_queue_.begin());
    }

    // Add relative pose from previous to current frames to the queue
    pose_transition_queue_.push_back(origin_pose_previous_.inverse() * origin_pose_current);
    // Update the previous pose
    origin_pose_previous_ = origin_pose_current;
}

float StochasticFilter::computePoseZScore(const Eigen::Matrix4f &origin_pose_previous, const Eigen::Matrix4f &origin_pose_current) const
{
    // Calculate the average predicted current pose using the transition queue and the previous pose
    Eigen::Matrix4f average_origin_pose_current = Eigen::Matrix4f::Zero();
    std::vector<Eigen::Vector3f> xyz_vector;
    xyz_vector.reserve(queue_size_);
    for (std::size_t i = 0; i < queue_size_; ++i)
    {
        const Eigen::Matrix4f temp_origin_pose_current(pose_transition_queue_[i]*origin_pose_previous);
        average_origin_pose_current += weights_[i] * temp_origin_pose_current;
        xyz_vector.emplace_back(Eigen::Vector3f(temp_origin_pose_current.block<3, 1>(0, 3)));
    }

    const Eigen::Vector3f average_xyz(average_origin_pose_current.block<3, 1>(0, 3));

    // Calculate the standard deviation of the predicted current pose
    // We will calculate the deviation of the xyz coordinates
    Eigen::Vector3f xyz_deviation = Eigen::Vector3f::Zero();
    for (std::size_t i = 0; i < queue_size_; ++i)
    {
        xyz_deviation += weights_[i] * (xyz_vector[i] - average_xyz).cwiseAbs();
    }
    // Threshold the deviation using the maximum traveled distance per scan acquisition
    xyz_deviation = xyz_deviation.cwiseMin(Eigen::Vector3f(max_distance_per_scan_/3.0f, max_distance_per_scan_/3.0f, max_distance_per_scan_/3.0f));

    // Calculate the absolute z score for each observed variable
    const Eigen::Vector3f z_scores = (origin_pose_current.block<3, 1>(0, 3) - average_xyz).cwiseAbs().cwiseQuotient(xyz_deviation);

    // Return the maximum value in the z scores
    return z_scores.maxCoeff();
}

Eigen::Matrix4f StochasticFilter::applyGaussianFilterToCurrentPose(const Eigen::Matrix4f &origin_pose_previous, const Eigen::Matrix4f &origin_pose_current) const
{
    // Calculate the z score of the current pose given the previous pose and the transition queue history
    const float pose_z_score = computePoseZScore(origin_pose_previous, origin_pose_current);

    // If the probability is below a threshold, we will apply the filter
    if (pose_z_score > n_std_dev_threshold_)
    {
        // Calculate the average predicted current pose using the transition queue and the previous pose
        Eigen::Matrix4f average_origin_pose_current = Eigen::Matrix4f::Zero();
        for (std::size_t i = 0; i < queue_size_; ++i)
        {
            average_origin_pose_current += weights_[i] * pose_transition_queue_[i]*origin_pose_previous;
        }

        return average_origin_pose_current;
    }

    return origin_pose_current;
}

inline float StochasticFilter::normalPDF(const float x, const float mean, const float stddev) const
{
    return std::exp(-std::pow(x - mean, 2) / (2.0f * std::pow(stddev, 2))) / std::sqrt(2.0f * M_PI * std::pow(stddev, 2));
}

inline float StochasticFilter::normalCDF(double x, double mean, double stddev) const
{
    return static_cast<float>(0.5 * (1 + std::erf((x - mean)/(stddev*std::sqrt(2)))));
}
