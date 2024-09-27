#include <localization/stochastic_filter.h>

StochasticFilter::StochasticFilter()
{
    odometry_poses_.reserve(covariance_filter_queue_size_);
}

void StochasticFilter::addPoseToOdometryQueue(const Eigen::Matrix4f &pose)
{
    // Add the pose in the circular vector for efficiency
    if (odometry_poses_.size() >= covariance_filter_queue_size_)
    {
        circular_vector_counter_ = (circular_vector_counter_ + 1) % covariance_filter_queue_size_;
        odometry_poses_[circular_vector_counter_] = pose;
    }
    else
    {
        odometry_poses_.emplace_back(pose);
        circular_vector_counter_ = static_cast<int>(odometry_poses_.size() - 1);
    }
}

Eigen::Matrix3f StochasticFilter::calculateCovarianceMatrix(const std::vector<Eigen::Matrix4f> &poses) const
{
    Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero();
    Eigen::Vector3f mean_pose = Eigen::Vector3f::Zero();
    
    for (const auto &pose : poses)
    {
        mean_pose += pose.block<3, 1>(0, 3);
    }
    mean_pose /= poses.size();

    for (const auto &pose : poses)
    {
        Eigen::Vector3f pose_diff = pose.block<3, 1>(0, 3) - mean_pose;
        covariance_matrix += pose_diff * pose_diff.transpose();
    }

    return covariance_matrix / poses.size();
}

void StochasticFilter::calculateCovarianceGains()
{
    // If not enough data, simply hand the biggest weight to odometry reading
    if (odometry_poses_.size() < minimum_poses_for_covariance_)
    {
        gps_gain_ = 0.1f;
        odometry_gain_ = 0.9f;
        return;
    }

    // Calculate the covariance matrices for the GPS and odometry poses
    const Eigen::Matrix3f odometry_covariance_matrix = calculateCovarianceMatrix(odometry_poses_);

    // Calculate the gains based on the trace of the covariance matrices
    const float gps_pose_weigth = gps_covariance_matrix_.trace();
    const float odometry_pose_weigth = odometry_covariance_matrix.trace();

    // Make the gains inverselly proportional to the trace of the covariance matrices
    const float weight_sum = gps_pose_weigth + odometry_pose_weigth;
    gps_gain_ = odometry_pose_weigth / weight_sum;
    odometry_gain_ = gps_pose_weigth / weight_sum;
}

void StochasticFilter::setGPSCovarianceMatrix(const Eigen::Matrix3f &gps_covariance)
{
    gps_covariance_matrix_ = gps_covariance;
}

float StochasticFilter::getGPSGain() const
{
    return gps_gain_;
}

float StochasticFilter::getOdometryGain() const
{
    return odometry_gain_;
}
