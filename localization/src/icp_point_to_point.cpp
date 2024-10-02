#include <localization/icp_point_to_point.h>

ICPPointToPoint::ICPPointToPoint(const float max_correspondence_dist, const int num_iterations, const float acceptable_mean_error, const float transformation_epsilon)
{
    max_correspondence_dist_ = max_correspondence_dist;
    num_iterations_ = num_iterations;
    acceptable_mean_error_ = acceptable_mean_error;
    transformation_epsilon_ = transformation_epsilon;

    last_error_ = std::numeric_limits<float>::max();
    initial_transform_ = Eigen::Matrix4f::Identity();

    source_cloud_pcl_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    target_cloud_pcl_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
}

void ICPPointToPoint::setMaxCorrespondenceDist(const float max_correspondence_dist)
{
    max_correspondence_dist_ = max_correspondence_dist;
}

void ICPPointToPoint::setNumIterations(const int num_iterations)
{
    num_iterations_ = num_iterations;
}

void ICPPointToPoint::setTransformationEpsilon(const float transformation_epsilon)
{
    transformation_epsilon_ = transformation_epsilon;
}

void ICPPointToPoint::setAcceptableMeanError(const float acceptable_error)
{
    acceptable_mean_error_ = acceptable_error;
}

void ICPPointToPoint::setInitialTransformation(const Eigen::Matrix4f &initial_transformation)
{
    initial_transform_ = initial_transformation;
}

void ICPPointToPoint::setDebugMode(bool debug_mode)
{
    debug_mode_ = debug_mode;
}

void ICPPointToPoint::setInputPointClouds(const pcl::PointCloud<PointT>::Ptr &source_cloud,
                                         const pcl::PointCloud<PointT>::Ptr &target_cloud)
{
    source_cloud_pcl_ = source_cloud;
    target_cloud_pcl_ = target_cloud;

    source_cloud_eigen_ = convertPclToEigen(source_cloud_pcl_);
}

void ICPPointToPoint::souceTargetCorrespondences(Eigen::MatrixX3f& source_cloud, Eigen::MatrixX3f& target_cloud) const
{
    // Init correspondences and kdtree for target cloud
    std::vector<std::pair<Eigen::Vector3f, PointT>> correspondences;
    correspondences.reserve(source_cloud.rows());
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(target_cloud_pcl_);

    // Look for valid correspondences in the target cloud
    for (int i = 0; i < source_cloud.rows(); ++i)
    {
        const PointT source_point(source_cloud(i, 0), source_cloud(i, 1), source_cloud(i, 2));
        std::vector<int> kdtree_point_indices(1);
        std::vector<float> kdtree_point_distances(1);
        kdtree.nearestKSearch(source_point, 1, kdtree_point_indices, kdtree_point_distances);
        if (!kdtree_point_indices.empty())
        {
            correspondences.emplace_back(std::make_pair(source_cloud.row(i), target_cloud_pcl_->points[kdtree_point_indices[0]]));
        }
    }

    // Reset the input clouds with the correspondences
    source_cloud.resize(correspondences.size(), 3);
    target_cloud.resize(correspondences.size(), 3);
    for (std::size_t i = 0; i < correspondences.size(); ++i)
    {
        source_cloud.row(i) = correspondences[i].first;
        target_cloud.row(i) = Eigen::Vector3f(correspondences[i].second.x, correspondences[i].second.y, correspondences[i].second.z);
    }
}

inline Eigen::MatrixX3f ICPPointToPoint::convertPclToEigen(const pcl::PointCloud<PointT>::Ptr &cloud) const
{
    Eigen::MatrixX3f cloud_eigen(cloud->size(), 3);
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        cloud_eigen(i, 0) = cloud->points[i].x;
        cloud_eigen(i, 1) = cloud->points[i].y;
        cloud_eigen(i, 2) = cloud->points[i].z;
    }

    return cloud_eigen;
}

inline void ICPPointToPoint::applyTransformation(const Eigen::Matrix4f &T, Eigen::MatrixX3f &cloud) const
{
    for (int i = 0; i < cloud.rows(); ++i)
    {
        const float transformed_x = T(0, 0) * cloud(i, 0) + T(0, 1) * cloud(i, 1) + T(0, 2) * cloud(i, 2) + T(0, 3);
        const float transformed_y = T(1, 0) * cloud(i, 0) + T(1, 1) * cloud(i, 1) + T(1, 2) * cloud(i, 2) + T(1, 3);
        const float transformed_z = T(2, 0) * cloud(i, 0) + T(2, 1) * cloud(i, 1) + T(2, 2) * cloud(i, 2) + T(2, 3);
        cloud(i, 0) = transformed_x;
        cloud(i, 1) = transformed_y;
        cloud(i, 2) = transformed_z;
    }
}

Eigen::Matrix4f ICPPointToPoint::calculateStepBestTransformation(const Eigen::MatrixX3f &source_cloud, 
                                                                const Eigen::MatrixX3f &target_cloud) const
{
    // Step 1: Compute centroids
    Eigen::Vector3f centroid_source(0, 0, 0), centroid_target(0, 0, 0);
    for (int i = 0; i < source_cloud.rows(); ++i)
    {
        centroid_source += source_cloud.row(i);
        centroid_target += target_cloud.row(i);
    }
    centroid_source /= source_cloud.rows();
    centroid_target /= target_cloud.rows();

    // Step 2: Subtract centroids to get zero-mean clouds
    Eigen::MatrixX3f source_zero_mean(source_cloud.rows(), 3), target_zero_mean(target_cloud.rows(), 3);
    for (int i = 0; i < source_cloud.rows(); ++i)
    {
        source_zero_mean.row(i) = source_cloud.row(i) - centroid_source.transpose();
        target_zero_mean.row(i) = target_cloud.row(i) - centroid_target.transpose();
    }
    
    // Step 3: Compute covariance matrix
    Eigen::Matrix3f H = source_zero_mean.transpose() * target_zero_mean;

    // Step 4: Compute SVD of covariance matrix
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();

    // Step 5: Compute rotation matrix
    Eigen::Matrix3f R_step = V * U.transpose();

    // Handle special reflection case (det(R) = -1)
    if (R_step.determinant() < 0) 
    {
        V.col(2) *= -1;
        R_step = V * U.transpose();
    }

    // Step 6: Compute translation vector
    const Eigen::Vector3f t_step = centroid_target - R_step * centroid_source;

    // Step 7: Create the transformation matrix
    Eigen::Matrix4f T_step(Eigen::Matrix4f::Identity());
    T_step.block<3, 3>(0, 0) = R_step;
    T_step.block<3, 1>(0, 3) = t_step;
    return T_step;
}

inline float ICPPointToPoint::calculateErrorMetric(const Eigen::MatrixX3f &source_cloud, const Eigen::MatrixX3f &target_cloud) const
{
    float error = 0.0f;
    for (int i = 0; i < source_cloud.rows(); ++i)
    {
        error += (source_cloud.row(i) - target_cloud.row(i)).norm();
    }

    return error/source_cloud.rows();
}

inline void ICPPointToPoint::printStepDebug(const int i, const float error) const
{
    std::cout << "[ICP INFO] Iteration " << i << " - Error: " << error << std::endl;
    if (error < acceptable_mean_error_)
    {
        std::cout << "[ICP INFO] Acceptable error reached. Stopping iterations." << std::endl;
    }
    if (std::abs(last_error_ - error) < transformation_epsilon_)
    {
        std::cout << "[ICP INFO] Transformation epsilon reached. Stopping iterations." << std::endl;
    }
}

Eigen::Matrix4f ICPPointToPoint::calculateAlignmentTransformation()
{
    // Apply transformation to the source cloud
    Eigen::MatrixX3f transformed_source_cloud(source_cloud_eigen_);
    applyTransformation(initial_transform_, transformed_source_cloud);
    // Get the correspondences in the target cloud
    Eigen::MatrixX3f correspondent_target_cloud;
    souceTargetCorrespondences(transformed_source_cloud, correspondent_target_cloud);
    if (transformed_source_cloud.rows() < 3)
    {
        std::cerr << "[ICP ERROR] Not enough valid correspondences found. Aborting." << std::endl;
        return initial_transform_;
    }

    // Iterate the algorithm
    Eigen::Matrix4f source_T_target = initial_transform_;
    int iterations_taken = 0;
    last_error_ = std::numeric_limits<float>::max();
    for (int i = 0; i < num_iterations_; ++i)
    {
        // Calculate the error metric
        const float error = calculateErrorMetric(transformed_source_cloud, correspondent_target_cloud);
        if (debug_mode_)
        {
            printStepDebug(i, error);
        }
        // Check if we reached the acceptable error
        if (error < acceptable_mean_error_)
        {
            break;
        }
        // If transformation is already very minimal, look for the correspondences again
        if (std::abs(last_error_ - error) < transformation_epsilon_)
        {
            souceTargetCorrespondences(transformed_source_cloud, correspondent_target_cloud);
        }
        // Compute the best transformation for the step
        const Eigen::Matrix4f T_step = calculateStepBestTransformation(transformed_source_cloud, correspondent_target_cloud);
        // Increment the transformation with the step result
        source_T_target = T_step * source_T_target;
        // Apply the transformation to the source cloud
        applyTransformation(T_step, transformed_source_cloud);
        // Update the error for next iteration
        last_error_ = error;
        // Update the number of iterations taken
        ++iterations_taken;
    }

    if (debug_mode_)
    {
        if (iterations_taken == num_iterations_)
        {
            std::cout << "[ICP INFO] We reached the maximum number of iterations. Returning best transform found." << std::endl;
        }
        std::cout << "[ICP INFO] Total iterations taken: " << iterations_taken << std::endl;
        std::cout << "[ICP INFO] Final error: " << last_error_ << std::endl;
        std::cout << "[ICP INFO] Final transformation matrix: " << std::endl << source_T_target << std::endl;
    }

    return source_T_target;
}
