#include "localization/brute_force_alignment.h"

BruteForceAlignment::BruteForceAlignment()
{
    map_T_sensor_previous_ = Eigen::Matrix4f::Identity();
    map_T_sensor_best_ = Eigen::Matrix4f::Identity();

    source_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
    target_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
}

void BruteForceAlignment::setXYZStep(const float x_step, const float y_step, const float z_step)
{
    x_step_ = x_step;
    y_step_ = y_step;
    z_step_ = z_step;
}

void BruteForceAlignment::setXYZRange(const float x, const float y, const float z)
{
    x_range_ = x;
    y_range_ = y;
    z_range_ = z;
}

void BruteForceAlignment::setRotationStep(const float yaw_step)
{
    yaw_step_ = yaw_step;
}

void BruteForceAlignment::setRotationRange(const float yaw)
{
    yaw_range_ = yaw;
}

void BruteForceAlignment::setMeanErrorThreshold(const float error_threshold)
{
    alignment_mean_error_threshold_ = error_threshold;
}

void BruteForceAlignment::setInitialGuess(const Eigen::Matrix4f &initial_guess)
{
    // Only update if we have not received any guess yet, otherwise we will deal with it inside the class
    if (map_T_sensor_previous_.trace() == 4.0f)
    {
        map_T_sensor_previous_ = initial_guess;
    }
}

void BruteForceAlignment::setSourceCloud(const pcl::PointCloud<PointT>::Ptr &cloud)
{
    source_cloud_ = cloud;
}

void BruteForceAlignment::setTargetCloud(const pcl::PointCloud<PointT>::Ptr &cloud)
{
    target_cloud_ = cloud;
}

void BruteForceAlignment::resetFirstAlignment(const bool value)
{
    first_alignment_completed_ = value;
}

bool BruteForceAlignment::alignClouds()
{
    // Initialize the best transformation
    Eigen::Matrix4f best_T = Eigen::Matrix4f::Identity();
    float best_score = std::numeric_limits<float>::max();

    // Create the KDtree
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(target_cloud_);

    // Create tests sequences
    std::vector<float> x_test, y_test, z_test, yaw_test;
    createTestTransformSequences(x_test, y_test, z_test, yaw_test);

    // Iterate over the possible transformations from combinations of the test sequences
    for (const auto &x : x_test)
    {
        for (const auto &y : y_test)
        {
            for (const auto &z : z_test)
            {
                for (const auto &yaw : yaw_test)
                {
                    // Create the transformation matrix
                    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
                    T.block<3, 3>(0, 0) = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
                    T.block<3, 1>(0, 3) = Eigen::Vector3f(x, y, z);
                    T = map_T_sensor_previous_*T;

                    // Calculate the score
                    float score = 0.0f;
                    for (const auto &p : source_cloud_->points)
                    {
                        const Eigen::Vector4f transf_point = T*Eigen::Vector4f(p.x, p.y, p.z, 1.0f);
                        const PointT transf_point_t(transf_point[0], transf_point[1], transf_point[2]);
                        std::vector<int> indices(1);
                        std::vector<float> distances(1);
                        kdtree.nearestKSearch(transf_point_t, 1, indices, distances);
                        score += distances[0];
                    }
                    score /= static_cast<float>(source_cloud_->size());

                    // Check if the score is better
                    if (score < best_score)
                    {
                        best_score = score;
                        best_T = T;
                    }
                    // If lower that the threshold we can stop the search
                    if (score < alignment_mean_error_threshold_)
                    {
                        map_T_sensor_best_ = T;
                        first_alignment_completed_ = true;
                        return true;
                    }
                }
            }
        }
    }

    // Update the previus transformation for next iteration
    map_T_sensor_previous_ = best_T;

    // Check if the first alignment was completed
    if (best_score < alignment_mean_error_threshold_)
    {
        map_T_sensor_best_ = best_T;
        first_alignment_completed_ = true;
        return true;
    }
    return false;
}

bool BruteForceAlignment::firstAlignmentCompleted() const
{
    return first_alignment_completed_;
}

Eigen::Matrix4f BruteForceAlignment::getBestTransformation() const
{
    return first_alignment_completed_ ? map_T_sensor_best_ : map_T_sensor_previous_;
}

void BruteForceAlignment::createTestTransformSequences(std::vector<float>& x_test, std::vector<float>& y_test, std::vector<float>& z_test, std::vector<float>& yaw_test)
{
    x_test.clear();
    y_test.clear();
    z_test.clear();
    yaw_test.clear();
    x_test.reserve(x_range_/x_step_ + 1);
    y_test.reserve(y_range_/y_step_ + 1);
    z_test.reserve(z_range_/z_step_ + 1);
    yaw_test.reserve(yaw_range_/yaw_step_ + 1);

    // Use a breadth-first search to create the sequences
    for (int i = 0; i < x_range_/(2*x_step_) + 1; ++i)
    {
        x_test.push_back(-i*x_step_);
        x_test.push_back(i*x_step_);
    }
    for (int i = 0; i < y_range_/(2*y_step_) + 1; ++i)
    {
        y_test.push_back(-i*y_step_);
        y_test.push_back(i*y_step_);
    }
    for (int i = 0; i < z_range_/(2*z_step_) + 1; ++i)
    {
        z_test.push_back(-i*z_step_);
        z_test.push_back(i*z_step_);
    }
    for (int i = 0; i < yaw_range_/(2*yaw_step_) + 1; ++i)
    {
        yaw_test.push_back(-i*yaw_step_);
        yaw_test.push_back(i*yaw_step_);
    }
}
