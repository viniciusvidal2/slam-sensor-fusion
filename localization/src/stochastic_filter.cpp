#include <localization/stochastic_filter.h>

StochasticFilter::StochasticFilter()
{
    gps_poses_.reserve(covariance_filter_queue_size_);
    odometry_poses_.reserve(covariance_filter_queue_size_);
}

void StochasticFilter::addPosesToQueues(const Eigen::Matrix4f &gps_pose, const Eigen::Matrix4f &odometry_pose)
{
    // Add the poses in the circular vectors for efficiency
    if (gps_poses_.size() >= covariance_filter_queue_size_)
    {
        circular_vector_counter_ = (circular_vector_counter_ + 1) % covariance_filter_queue_size_;
        gps_poses_[circular_vector_counter_] = gps_pose;
        odometry_poses_[circular_vector_counter_] = odometry_pose;
    }
    else
    {
        gps_poses_.emplace_back(gps_pose);
        odometry_poses_.emplace_back(odometry_pose);
        circular_vector_counter_ = static_cast<int>(gps_poses_.size() - 1);
    }

        // std::cout << "gps poses so far: " << gps_poses_.size() << std::endl;
        // for (const auto &pose : gps_poses_)
        // {
        //     std::cout << pose << std::endl;
        //     std::cout << "-------------------" << std::endl;
        // }
}

Eigen::Matrix3f StochasticFilter::calculateCovarianceMatrix(const std::vector<Eigen::Matrix4f> &poses)
{
    Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero();
    Eigen::Vector3f mean_pose = Eigen::Vector3f::Zero();
    for (const auto &pose : poses)
    {
        mean_pose += pose.block<3, 1>(0, 3);
    }
    mean_pose /= poses.size();
    std::cout << "MEAN POSE: \n" << mean_pose << std::endl;

    for (const auto &pose : poses)
    {
        Eigen::Vector3f pose_diff = pose.block<3, 1>(0, 3) - mean_pose;
        covariance_matrix += pose_diff * pose_diff.transpose();
    }

    return covariance_matrix / poses.size();
}

void StochasticFilter::calculateCovarianceGains(float &gps_pose_gain, float &odometry_pose_gain)
{
    // If not enough data, simply hand the biggest weight to odometry reading
    if (gps_poses_.size() < minimum_poses_for_covariance_)
    {
        gps_pose_gain = 0.1f;
        odometry_pose_gain = 0.9f;
        return;
    }

    // Calculate the covariance matrices for the GPS and odometry poses
    const Eigen::Matrix3f gps_covariance = calculateCovarianceMatrix(gps_poses_);
    std::cout << "GPS COVARIANCE: \n" << gps_covariance << std::endl;
    const Eigen::Matrix3f odometry_covariance = calculateCovarianceMatrix(odometry_poses_);
    std::cout << "ODOMETRY COVARIANCE: \n" << odometry_covariance << std::endl;

    // Calculate the gains based on the trace of the covariance matrices
    const float gps_pose_weigth = gps_covariance.trace();
    const float odometry_pose_weigth = odometry_covariance.trace();

    // Make the gains inverselly proportional to the trace of the covariance matrices
    const float weight_sum = gps_pose_weigth + odometry_pose_weigth;
    gps_pose_gain = odometry_pose_weigth / weight_sum;
    odometry_pose_gain = gps_pose_weigth / weight_sum;
}
