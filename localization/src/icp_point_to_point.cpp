#include <localization/icp_point_to_point.h>

ICPPointToPoint::ICPPointToPoint(const float max_correspondence_dist, const int num_iterations, const float acceptable_mean_error, const float transformation_epsilon)
{
    max_correspondence_dist_ = max_correspondence_dist;
    num_iterations_ = num_iterations;
    acceptable_mean_error_ = acceptable_mean_error;
    transformation_epsilon_ = transformation_epsilon;

    current_error_ = std::numeric_limits<float>::max();
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

void ICPPointToPoint::filterValidCorrespondencesInClouds(Eigen::MatrixX3f &source_cloud, Eigen::MatrixX3f &target_cloud)
{
    // Filter valid correspondences (where both source and target are nonzero)
    std::vector<Eigen::Vector3f> valid_source_correspondences_points, valid_target_correspondences_points;
    valid_source_correspondences_points.reserve(source_cloud.rows());
    valid_target_correspondences_points.reserve(target_cloud.rows());
    for (int i = 0; i < source_cloud.rows(); ++i)
    {
        if (target_cloud(i, 0) != 0.0f && target_cloud(i, 1) != 0.0f && target_cloud(i, 2) != 0.0f)
        {
            valid_source_correspondences_points.emplace_back(source_cloud.row(i));
            valid_target_correspondences_points.emplace_back(target_cloud.row(i));
        }
    }
    source_cloud.resize(valid_source_correspondences_points.size(), 3);
    target_cloud.resize(valid_target_correspondences_points.size(), 3);
    for (std::size_t i = 0; i < valid_source_correspondences_points.size(); ++i)
    {
        source_cloud.row(i) = valid_source_correspondences_points[i];
        target_cloud.row(i) = valid_target_correspondences_points[i];
    }
}

Eigen::MatrixX3f ICPPointToPoint::findSourceCorrespondencesInTargetCloud(const Eigen::MatrixX3f& source_cloud)
{
    Eigen::MatrixX3f correspondent_target_cloud(source_cloud.rows(), 3);
    correspondent_target_cloud.setZero();
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(target_cloud_pcl_);

    for (int i = 0; i < source_cloud.rows(); ++i)
    {
        PointT source_point;
        source_point.x = source_cloud(i, 0);
        source_point.y = source_cloud(i, 1);
        source_point.z = source_cloud(i, 2);
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        if (kdtree.nearestKSearch(source_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            if (pointNKNSquaredDistance[0] < max_correspondence_dist_)
            {
                correspondent_target_cloud(i, 0) = target_cloud_pcl_->points[pointIdxNKNSearch[0]].x;
                correspondent_target_cloud(i, 1) = target_cloud_pcl_->points[pointIdxNKNSearch[0]].y;
                correspondent_target_cloud(i, 2) = target_cloud_pcl_->points[pointIdxNKNSearch[0]].z;
            }
        }
    }

    return correspondent_target_cloud;
}

Eigen::MatrixX3f ICPPointToPoint::convertPclToEigen(const pcl::PointCloud<PointT>::Ptr &cloud)
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

Eigen::MatrixX3f ICPPointToPoint::applyTransformation(const Eigen::Matrix4f &transformation, const Eigen::MatrixX3f &cloud)
{
    Eigen::MatrixX3f transformed_cloud(cloud.rows(), 3);
    for (int i = 0; i < cloud.rows(); ++i)
    {
        transformed_cloud(i, 0) = transformation(0, 0) * cloud(i, 0) + transformation(0, 1) * cloud(i, 1) + transformation(0, 2) * cloud(i, 2) + transformation(0, 3);
        transformed_cloud(i, 1) = transformation(0, 0) * cloud(i, 0) + transformation(0, 1) * cloud(i, 1) + transformation(0, 2) * cloud(i, 2) + transformation(0, 3);
        transformed_cloud(i, 2) = transformation(0, 0) * cloud(i, 0) + transformation(0, 1) * cloud(i, 1) + transformation(0, 2) * cloud(i, 2) + transformation(0, 3);
    }

    return transformed_cloud;
}

Eigen::Matrix4f ICPPointToPoint::calculateStepBestTransformation(const Eigen::MatrixX3f &source_cloud, 
                                                                const Eigen::MatrixX3f &target_cloud)
{
    // Step 1: Compute centroids
    const Eigen::Vector3f centroid_source = source_cloud.colwise().mean();
    const Eigen::Vector3f centroid_target = target_cloud.colwise().mean();

    // Step 2: Subtract centroids to get zero-mean clouds
    const Eigen::MatrixX3f source_zero_mean = source_cloud.rowwise() - centroid_source.transpose();
    const Eigen::MatrixX3f target_zero_mean = target_cloud.rowwise() - centroid_target.transpose();

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

float ICPPointToPoint::calculateErrorMetric(const Eigen::MatrixX3f &source_cloud, const Eigen::MatrixX3f &target_cloud)
{
    float error = 0.0f;
    for (int i = 0; i < source_cloud.rows(); ++i)
    {
        error += (source_cloud.row(i) - target_cloud.row(i)).norm();
    }

    return error/source_cloud.rows();
}

Eigen::Matrix4f ICPPointToPoint::calculateAlignmentTransformation()
{
    Eigen::Matrix4f target_T_source = initial_transform_;
    int iterations_taken = 0;
    for (int i = 0; i < num_iterations_; ++i)
    {
        // Apply transformation to the source cloud
        Eigen::MatrixX3f transformed_source_cloud = applyTransformation(target_T_source, source_cloud_eigen_);
        // Get the correspondences in the target cloud
        Eigen::MatrixX3f correspondent_target_cloud = findSourceCorrespondencesInTargetCloud(transformed_source_cloud);
        // Create a sample point cloud with 6 points for source and cloud
        transformed_source_cloud.resize(6, 3);
        correspondent_target_cloud.resize(6, 3);
        transformed_source_cloud << 0.0f, 0.0f, 0.0f,
                               1.0f, 0.0f, 0.0f,
                               0.0f, 1.0f, 0.0f,
                               0.0f, 0.0f, 1.0f,
                               1.0f, 1.0f, 0.0f,
                               1.0f, 0.0f, 1.0f;
        correspondent_target_cloud << 0.0f, 0.0f, 0.0f,
                               2.0f, 0.0f, 0.0f,
                               0.0f, 2.0f, 0.0f,
                               0.0f, 0.0f, 2.0f,
                               2.0f, 2.0f, 0.0f,
                               2.0f, 0.0f, 2.0f;
        // Filter for valid correspondences
        filterValidCorrespondencesInClouds(transformed_source_cloud, correspondent_target_cloud);
        std::cout << "Source cloud size: " << transformed_source_cloud.rows() << " - Correspondent target cloud size: " << correspondent_target_cloud.rows() << std::endl;
        if (transformed_source_cloud.rows() < 3)
        {
            std::cerr << "[ICP ERROR] Not enough valid correspondences found. Aborting." << std::endl;
            break;
        }

        // Calculate the error metric
        const float error = calculateErrorMetric(transformed_source_cloud, correspondent_target_cloud);
        if (debug_mode_)
        {
            std::cout << "[ICP INFO] Iteration " << i << " - Error: " << error << std::endl;
        }
        if (error < acceptable_mean_error_)
        {
            std::cout << "[ICP INFO] Acceptable error reached. Stopping iterations." << std::endl;
            break;
        }
        if (std::abs(current_error_ - error) < transformation_epsilon_)
        {
            std::cout << "[ICP INFO] Transformation epsilon reached. Stopping iterations." << std::endl;
            std::cout << "[ICP DEBUG] Current error: " << current_error_ << " - New error: " << error << std::endl;
            break;
        }

        // Compute the best transformation for the step
        const Eigen::Matrix4f T_step = calculateStepBestTransformation(transformed_source_cloud, correspondent_target_cloud);
        // Increment the transformation with the step result
        target_T_source = T_step * target_T_source;
        std::cout << "[ICP DEBUG] Transformation matrix: " << std::endl << target_T_source << std::endl;
        // Update the error for next iteration
        current_error_ = error;
        // Update the number of iterations taken
        ++iterations_taken;
    }

    if (debug_mode_ && iterations_taken == num_iterations_)
    {
        std::cout << "[ICP INFO] We reached the maximum number of iterations with no convergence. Returning best transform found." << std::endl;
    }

    return target_T_source;
}
