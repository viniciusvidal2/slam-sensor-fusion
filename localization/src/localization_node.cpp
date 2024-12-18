#include "localization/localization_node.h"

LocalizationNode::LocalizationNode(ros::NodeHandle nh)
{
    // Parameters
    ros::NodeHandle pnh("~");
    pnh.param("/debug/enable", debug_, false);
    pnh.param("/map_data/home_relative_path", relative_folder_path_, static_cast<std::string>("Desktop/map_data"));
    pnh.param("/map_data/name", map_name_, static_cast<std::string>("map"));
    pnh.param("/map_data/max_map_optimization_poses", max_map_optimization_poses_, 50);
    pnh.param("/map_data/voxel_resolution", map_voxel_size_, 0.1f);
    pnh.param("/map_data/ref_frame_distance", ref_frame_distance_, 3.0f);
    pnh.param("/icp/num_iterations", icp_iterations_, 16);
    pnh.param("/icp/transformation_epsilon", icp_transform_epsilon_, 1e-8f);
    pnh.param("/icp/max_correspondence_dist", icp_max_correspondence_dist_, 0.05f);
    pnh.param("/icp/mean_accepted_error", icp_mean_accepted_error_, 0.05f);
    pnh.param("/velocity_filter/max_velocity", max_rover_velocity_, 1.6f);
    pnh.param("/pose_gains_calculation/fixed", pose_gains_calculation_option_, false);
    pnh.param("/pose_gains_calculation/odom_fixed_gain", odom_fixed_gain_, 0.95f);
    pnh.param("/pose_gains_calculation/gps_fixed_gain", gps_fixed_gain_, 0.05f);

    // Vehicle extrinsics params
    float vehicle_x_lidar, vehicle_y_lidar, vehicle_z_lidar;
    float vehicle_roll_lidar, vehicle_pitch_lidar, vehicle_yaw_lidar;
    pnh.param("/lidar_extrinsics/x_lidar_vehicle", vehicle_x_lidar, 0.0f);
    pnh.param("/lidar_extrinsics/y_lidar_vehicle", vehicle_y_lidar, 0.0f);
    pnh.param("/lidar_extrinsics/z_lidar_vehicle", vehicle_z_lidar, 0.0f);
    pnh.param("/lidar_extrinsics/roll_lidar_vehicle", vehicle_roll_lidar, 0.0f);
    pnh.param("/lidar_extrinsics/pitch_lidar_vehicle", vehicle_pitch_lidar, 0.0f);
    pnh.param("/lidar_extrinsics/yaw_lidar_vehicle", vehicle_yaw_lidar, 0.0f);
    Eigen::Matrix4f vehicle_T_lidar = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f vehicle_R_lidar;
    vehicle_R_lidar = Eigen::AngleAxisf(M_PI / 180.0f * vehicle_roll_lidar, Eigen::Vector3f::UnitX()) *
                      Eigen::AngleAxisf(M_PI / 180.0f * vehicle_pitch_lidar, Eigen::Vector3f::UnitY()) *
                      Eigen::AngleAxisf(M_PI / 180.0f * vehicle_yaw_lidar, Eigen::Vector3f::UnitZ());
    vehicle_T_lidar.block<3, 3>(0, 0) = vehicle_R_lidar;
    vehicle_T_lidar(0, 3) = vehicle_x_lidar;
    vehicle_T_lidar(1, 3) = vehicle_y_lidar;
    vehicle_T_lidar(2, 3) = vehicle_z_lidar;
    lidar_T_vehicle_ = vehicle_T_lidar.inverse();

    // Init the map point cloud and transformation with the frames manager
    global_map_frames_manager_ = std::make_shared<GlobalMapFramesManager>(std::string(std::getenv("HOME")) + "/" + relative_folder_path_,
                                                                          map_name_,
                                                                          static_cast<std::size_t>(max_map_optimization_poses_));
    map_cloud_ = global_map_frames_manager_->getMapCloud(map_voxel_size_);
    if (map_cloud_->empty())
    {
        ROS_ERROR("Could not get the map point cloud, exiting.");
        return;
    }
    applyUniformSubsample(map_cloud_, 3);
    map_T_global_ = global_map_frames_manager_->getMapTGlobal();
    if (map_T_global_ == Eigen::Matrix4d::Identity())
    {
        ROS_ERROR("Could not get the map_T_global transformation, exiting.");
        return;
    }

    // Init the ICP object to compute Point to Point alignment
    icp_ = std::make_shared<ICPPointToPoint>(icp_max_correspondence_dist_, icp_iterations_,
                                             icp_mean_accepted_error_, icp_transform_epsilon_);
    icp_->setDebugMode(debug_);

    // Reference transforms
    map_T_sensor_ = lidar_T_vehicle_ * Eigen::Matrix4f::Identity();
    odom_T_sensor_previous_ = lidar_T_vehicle_ * Eigen::Matrix4f::Identity();
    map_T_ref_ = lidar_T_vehicle_ * Eigen::Matrix4f::Identity();
    map_T_odom_ = lidar_T_vehicle_ * Eigen::Matrix4f::Identity();

    // Init the cropped map in the ref frame
    ref_cropped_map_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());

    // Create the publishers for the odometry and the point cloud
    map_T_sensor_pub_ = nh.advertise<nav_msgs::Odometry>("/localization/map_T_sensor", 10);
    map_T_sensor_prior_pub_ = nh.advertise<nav_msgs::Odometry>("/localization/map_T_sensor_prior", 10);
    map_T_sensor_odom_pub_ = nh.advertise<nav_msgs::Odometry>("/localization/map_T_sensor_odom", 10);
    map_T_sensor_gps_pub_ = nh.advertise<nav_msgs::Odometry>("/localization/map_T_sensor_gps", 10);
    cropped_scan_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/localization/cropped_scan_map_frame", 10);
    map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/localization/map", 10);

    // Compass subscriber, will be used to get the yaw angle
    compass_subscription_ = nh.subscribe<std_msgs::Float64>(
        "/mavros/global_position/compass_hdg",
        10, &LocalizationNode::compassCallback, this);

    // Initialize synchronized subscribers
    pointcloud_sub_.subscribe(nh, "/lidar_odometry/cloud_registered_body", 3);
    gps_sub_.subscribe(nh, "/mavros/global_position/global", 3);
    odom_sub_.subscribe(nh, "/lidar_odometry/odometry", 3);
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
        SyncPolicy(3), pointcloud_sub_, gps_sub_, odom_sub_));
    sync_->registerCallback(boost::bind(&LocalizationNode::localizationCallback, this, _1, _2, _3));

    ROS_INFO("Localization node initialized!");
}

void LocalizationNode::compassCallback(const std_msgs::Float64::ConstPtr &msg)
{
    // Invert the yaw based on Ardupilot convention that clockwise is positive
    current_compass_yaw_ = (90.0 - msg->data) * M_PI / 180.0;
    // Make sure the yaw is in the range -M_PI to M_PI
    if (current_compass_yaw_ > M_PI)
    {
        current_compass_yaw_ -= 2 * M_PI;
    }
    else if (current_compass_yaw_ < -M_PI)
    {
        current_compass_yaw_ += 2 * M_PI;
    }
}

inline void LocalizationNode::computePosePredictionFromOdometry(const nav_msgs::Odometry::ConstPtr &odom_msg,
                                                                Eigen::Matrix4f &odom_T_sensor_current,
                                                                Eigen::Matrix4f &map_T_sensor_current_odom) const
{
    // Get the current pose of the sensor in the odometry frame
    Eigen::Quaternionf odom_q_sensor_current(odom_msg->pose.pose.orientation.w,
                                             odom_msg->pose.pose.orientation.x,
                                             odom_msg->pose.pose.orientation.y,
                                             odom_msg->pose.pose.orientation.z);
    Eigen::Vector3f odom_t_sensor_current(odom_msg->pose.pose.position.x,
                                          odom_msg->pose.pose.position.y,
                                          odom_msg->pose.pose.position.z);
    odom_T_sensor_current.setIdentity();
    odom_T_sensor_current.block<3, 3>(0, 0) = odom_q_sensor_current.toRotationMatrix();
    odom_T_sensor_current.block<3, 1>(0, 3) = odom_t_sensor_current;
    // Apply vehicle extrinsics
    odom_T_sensor_current = lidar_T_vehicle_ * odom_T_sensor_current;

    // Calculate the previous_T_current transformation matrix
    const Eigen::Matrix4f previous_T_current(odom_T_sensor_previous_.inverse() * odom_T_sensor_current);

    // Pose prediction in map frame using the relative pose found in odom frame between previous and current frame readings
    map_T_sensor_current_odom = map_T_sensor_ * previous_T_current;
}

const Eigen::Matrix4f LocalizationNode::computeGpsCoarsePoseInMapFrame(const sensor_msgs::NavSatFix::ConstPtr &gps_msg) const
{
    // Convert the compass yaw to a rotation matrix
    Eigen::Matrix3f global_R_sensor;
    global_R_sensor = Eigen::AngleAxisf(current_compass_yaw_, Eigen::Vector3f::UnitZ());
    // Convert the GPS latitude, longitude and altitude to UTM coordinates
    double utm_northing, utm_easting;
    UTM::LLtoUTM(gps_msg->latitude, gps_msg->longitude, utm_northing, utm_easting);
    // Get the altitude from the table we got during mapping
    const float table_altitude = global_map_frames_manager_->getClosestAltitude(gps_msg->latitude, gps_msg->longitude);
    // Calculate the pose in map frame from the pose in global frame
    Eigen::Matrix4f global_T_sensor(Eigen::Matrix4f::Identity());
    global_T_sensor.block<3, 3>(0, 0) = global_R_sensor;
    global_T_sensor.block<3, 1>(0, 3) = Eigen::Vector3f(utm_easting, utm_northing, table_altitude);

    return lidar_T_vehicle_ * map_T_global_.cast<float>() * global_T_sensor;
}

inline nav_msgs::Odometry LocalizationNode::buildNavOdomMsg(const Eigen::Matrix4f &T,
                                                            const std::string &frame_id,
                                                            const std::string &child_frame_id,
                                                            const ros::Time &stamp) const
{
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = frame_id;
    odom_msg.child_frame_id = child_frame_id;
    odom_msg.pose.pose.position.x = static_cast<double>(T(0, 3));
    odom_msg.pose.pose.position.y = static_cast<double>(T(1, 3));
    odom_msg.pose.pose.position.z = static_cast<double>(T(2, 3));
    const Eigen::Quaternionf q(T.block<3, 3>(0, 0));
    odom_msg.pose.pose.orientation.w = static_cast<double>(q.w());
    odom_msg.pose.pose.orientation.x = static_cast<double>(q.x());
    odom_msg.pose.pose.orientation.y = static_cast<double>(q.y());
    odom_msg.pose.pose.orientation.z = static_cast<double>(q.z());

    return odom_msg;
}

void LocalizationNode::computePoseGainsFromCovarianceMatrices(const sensor_msgs::NavSatFix::ConstPtr &gps_msg,
                                                              const nav_msgs::Odometry::ConstPtr &odom_msg,
                                                              float &odom_gain, float &gps_gain) const
{
    // If fixed just send constant value to the gains with more value to odometry
    if (pose_gains_calculation_option_)
    {
        odom_gain = odom_fixed_gain_;
        gps_gain = gps_fixed_gain_;
        return;
    }

    // Get covariance matrices from the messages
    Eigen::Matrix3f gps_covariance_matrix;
    gps_covariance_matrix << gps_msg->position_covariance[0], gps_msg->position_covariance[1], gps_msg->position_covariance[2],
        gps_msg->position_covariance[3], gps_msg->position_covariance[4], gps_msg->position_covariance[5],
        gps_msg->position_covariance[6], gps_msg->position_covariance[7], gps_msg->position_covariance[8];
    Eigen::Matrix3f odom_covariance_matrix;
    odom_covariance_matrix << odom_msg->pose.covariance[0], odom_msg->pose.covariance[1], odom_msg->pose.covariance[2],
        odom_msg->pose.covariance[6], odom_msg->pose.covariance[7], odom_msg->pose.covariance[8],
        odom_msg->pose.covariance[12], odom_msg->pose.covariance[13], odom_msg->pose.covariance[14];
    // Calculate the trace of the covariance matrices as weights
    const float odom_weight = odom_covariance_matrix.trace();
    const float gps_weight = gps_covariance_matrix.trace();
    // Calculate the gains as inverse of the traces weights
    const float total_det = odom_weight + gps_weight;
    odom_gain = gps_weight / total_det;
    gps_gain = odom_weight / total_det;
}

void LocalizationNode::velocityFilter(Eigen::Matrix4f &pose,
                                      const Eigen::Matrix4f &previous_pose,
                                      const float time_diff) const
{
    // Calculate the velocity
    const Eigen::Vector3f t = pose.block<3, 1>(0, 3) - previous_pose.block<3, 1>(0, 3);
    const Eigen::Vector3f velocity_vector = t / time_diff;
    // If the velocity is too high, filter it to max_rover_velocity_
    const float velocity_value = velocity_vector.norm();
    if (velocity_value > max_rover_velocity_)
    {
        pose.block<3, 1>(0, 3) = previous_pose.block<3, 1>(0, 3) + max_rover_velocity_ / velocity_value * t;
    }
}

void LocalizationNode::initializePosesWithFirstReading(const sensor_msgs::NavSatFix::ConstPtr &gps_msg,
                                                       const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    // Set the map_T_sensor based on the first GPS reading
    map_T_sensor_ = computeGpsCoarsePoseInMapFrame(gps_msg);
    map_T_ref_ = map_T_sensor_;
    // Set previous odometry transformation with odometry reading
    Eigen::Quaternionf odom_q_sensor_previous(odom_msg->pose.pose.orientation.w,
                                              odom_msg->pose.pose.orientation.x,
                                              odom_msg->pose.pose.orientation.y,
                                              odom_msg->pose.pose.orientation.z);
    Eigen::Vector3f odom_t_sensor_previous(odom_msg->pose.pose.position.x,
                                           odom_msg->pose.pose.position.y,
                                           odom_msg->pose.pose.position.z);
    odom_T_sensor_previous_.setIdentity();
    odom_T_sensor_previous_.block<3, 3>(0, 0) = odom_q_sensor_previous.toRotationMatrix();
    odom_T_sensor_previous_.block<3, 1>(0, 3) = odom_t_sensor_previous;
    // Apply vehicle extrinsics
    odom_T_sensor_previous_ = lidar_T_vehicle_ * odom_T_sensor_previous_;
}

bool LocalizationNode::performCoarseAlignment(const pcl::PointCloud<PointT>::Ptr &scan_cloud,
                                              const pcl::PointCloud<PointT>::Ptr &map_cloud)
{
    // Perform first alignment if not aligned yet to the start of the trip in map frame
    pcl::PointCloud<PointT>::Ptr map_cloud_temp = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    *map_cloud_temp = *map_cloud;
    pcl::PointCloud<PointT>::Ptr scan_cloud_temp = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    *scan_cloud_temp = *scan_cloud;
    removeFloor(map_cloud_temp);
    removeFloor(scan_cloud_temp);

    // First NDT registration
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    ndt.setMaximumIterations(50);
    ndt.setResolution(1.0);
    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(0.1);
    ndt.setInputSource(scan_cloud_temp);
    ndt.setInputTarget(map_cloud_temp);
    pcl::PointCloud<PointT> output;
    ndt.align(output, map_T_sensor_);
    map_T_sensor_ = ndt.getFinalTransformation();

    // Create sequences to try ICP
    std::vector<float> max_corresp_distances = {5.0f, 4.0f, 3.0f, 2.0f, 1.0f, 0.5f};
    std::vector<float> acceptable_mean_errors = {0.3f, 0.2f};
    // Apply a heavy ICP to get the initial alignment
    ROS_WARN("Running brute force with ICP.");
    float best_error = 1e6f;
    icp_->setNumIterations(100);
    for (const auto &max_corresp_dist : max_corresp_distances)
    {
        for (const auto &acceptable_mean_error : acceptable_mean_errors)
        {
            icp_->setMaxCorrespondenceDist(max_corresp_dist);
            icp_->setAcceptableMeanError(acceptable_mean_error);
            icp_->setSourcePointCloud(scan_cloud_temp);
            icp_->setTargetPointCloud(map_cloud_temp);
            icp_->setInitialTransformation(map_T_sensor_);
            const auto icp_result = icp_->calculateAlignment();
            if (icp_result.error < best_error)
            {
                best_error = icp_result.error;
                map_T_sensor_ = icp_result.transformation;
            }
            if (icp_result.has_converged)
            {
                ROS_WARN("ICP alignment successfull with max_corresp_dist: %f and acceptable_mean_error: %f",
                         max_corresp_dist, acceptable_mean_error);
                icp_->setMaxCorrespondenceDist(icp_max_correspondence_dist_);
                icp_->setTransformationEpsilon(icp_transform_epsilon_);
                icp_->setAcceptableMeanError(icp_mean_accepted_error_);
                icp_->setNumIterations(icp_iterations_);
                map_T_sensor_ = icp_result.transformation;
                map_T_odom_ = map_T_sensor_;
                coarse_alignment_complete_ = true;
                return true;
            }
        }
    }

    ROS_WARN("ICP alignment not successfull, will try again next time.");
    return false;
}

void LocalizationNode::filterVehicleBox(pcl::PointCloud<PointT> &cloud)
{
    // Temp cloud to store points outside of vehicle box
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    for (const auto &point : cloud.points)
    {
        if (point.x < -vehicle_box_size_.x() / 2.0 || point.x > vehicle_box_size_.x() / 2.0 ||
            point.y < -vehicle_box_size_.y() / 2.0 || point.y > vehicle_box_size_.y() / 2.0 ||
            point.z > vehicle_box_size_.z())
        {
            cloud_filtered->push_back(point);
        }
    }
    // Update the input cloud
    cloud = *cloud_filtered;
}

void LocalizationNode::localizationCallback(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg,
                                            const sensor_msgs::NavSatFix::ConstPtr &gps_msg,
                                            const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    ///////////////////////////////////////// NODE STARTUP /////////////////////////////////////////
    // If the altitude is still wrong, we cannot proceed
    if (gps_msg->altitude < 0)
    {
        if (debug_)
        {
            ROS_WARN("Altitude is still not correct, waiting for a valid GPS message.");
        }
        return;
    }

    if (first_time_)
    {
        initializePosesWithFirstReading(gps_msg, odom_msg);
        previous_odom_stamp_ = odom_msg->header.stamp;
        first_time_ = false;
        return;
    }

    ///////////////////////////////////////// PREPROCESSING /////////////////////////////////////////
    // Start timer to measure
    auto start = std::chrono::high_resolution_clock::now();

    // Convert the incoming point cloud and subsample
    pcl::PointCloud<PointT>::Ptr scan_cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*pointcloud_msg, *scan_cloud);
    applyUniformSubsample(scan_cloud, 2);
    // Apply filter to remove vehicle points
    filterVehicleBox(*scan_cloud);

    // Crop the input scan around the sensor frame origin
    pcl::PointCloud<PointT>::Ptr cropped_scan_cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    cropPointCloudThroughRadius(Eigen::Matrix4f::Identity(), cloud_crop_radius_, scan_cloud, cropped_scan_cloud);
    // Crop the map point cloud around the sensor position in map frame
    // Only do it if we have a new reference frame measured by walked distance
    const Eigen::Matrix4f sensor_T_ref = map_T_sensor_.inverse() * map_T_ref_;
    if (sensor_T_ref.block<3, 1>(0, 3).norm() > ref_frame_distance_ || ref_cropped_map_cloud_->empty())
    {
        cropPointCloudThroughRadius(map_T_sensor_, cloud_crop_radius_, map_cloud_, ref_cropped_map_cloud_);
        icp_->setTargetPointCloud(ref_cropped_map_cloud_);
        map_T_ref_ = map_T_sensor_;
    }

    ///////////////////////////////////////// COARSE ALIGNMENT /////////////////////////////////////////
    // Try to perform coarse alignment, if we are successfull we can proceed to fine alignment
    if (!coarse_alignment_complete_)
    {
        if (!performCoarseAlignment(cropped_scan_cloud, ref_cropped_map_cloud_))
        {
            ROS_WARN("Coarse alignment not successfull, localization pose might not be precise.");
        }
    }

    ///////////////////////////////////////// FINE ALIGNMENT /////////////////////////////////////////
    // Timestamp difference from last messages to apply filters
    const float time_diff = (odom_msg->header.stamp - previous_odom_stamp_).toSec();

    // Obtain the odometry prediction for the pose both in map and odom frames
    Eigen::Matrix4f odom_T_sensor_current;
    Eigen::Matrix4f map_T_sensor_odom;
    computePosePredictionFromOdometry(odom_msg, odom_T_sensor_current, map_T_sensor_odom);

    // Obtain the coarse pose from GPS and compass in the map frame, based on the global frame information
    const Eigen::Matrix4f map_T_sensor_gps = computeGpsCoarsePoseInMapFrame(gps_msg);

    // Obtain the weighted coarse pose from GPS and Odometry fusion based on covariance
    float gps_compass_gain, odometry_gain;
    computePoseGainsFromCovarianceMatrices(gps_msg, odom_msg, odometry_gain, gps_compass_gain);
    Eigen::Matrix4f map_T_sensor_prior = odometry_gain * map_T_sensor_odom + gps_compass_gain * map_T_sensor_gps;
    velocityFilter(map_T_sensor_prior, map_T_sensor_, time_diff);

    // Align the point clouds with ICP (will return transformation in map frame)
    icp_->setSourcePointCloud(cropped_scan_cloud);
    icp_->setInitialTransformation(map_T_sensor_prior);
    const auto icp_result = icp_->calculateAlignment();
    // Filter the ICP output based on velocity before updating the sensor pose in map frame
    Eigen::Matrix4f map_T_sensor_fine = icp_result.transformation;
    velocityFilter(map_T_sensor_fine, map_T_sensor_, time_diff);
    map_T_sensor_ = map_T_sensor_fine;

    // Update the transformation in odom frame and timestamp
    odom_T_sensor_previous_ = odom_T_sensor_current;
    previous_odom_stamp_ = odom_msg->header.stamp;

    // Publish the localized pose in map frame
    map_T_sensor_pub_.publish(buildNavOdomMsg(map_T_sensor_, "map", "sensor", odom_msg->header.stamp));

    /////////////////////////////////////////////////////////////////////////////////////////////////////

    if (debug_)
    {
        // Log the time taken to process the callback
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        ROS_INFO("CALLBACK TOOK %f seconds", elapsed.count());
        // Publish debug poses
        map_T_sensor_odom = map_T_odom_ * odom_T_sensor_current;
        map_T_sensor_odom_pub_.publish(buildNavOdomMsg(map_T_sensor_odom, "map", "sensor", odom_msg->header.stamp));
        map_T_sensor_gps_pub_.publish(buildNavOdomMsg(map_T_sensor_gps, "map", "sensor", odom_msg->header.stamp));
        map_T_sensor_prior_pub_.publish(buildNavOdomMsg(map_T_sensor_prior, "map", "sensor", odom_msg->header.stamp));
        // Publish the cropped scan
        pcl::transformPointCloud(*cropped_scan_cloud, *cropped_scan_cloud, map_T_sensor_);
        cropped_scan_cloud->header.frame_id = "map";
        sensor_msgs::PointCloud2 cropped_scan_msg;
        pcl::toROSMsg(*cropped_scan_cloud, cropped_scan_msg);
        cropped_scan_msg.header = pointcloud_msg->header;
        cropped_scan_msg.header.frame_id = "map";
        cropped_scan_pub_.publish(cropped_scan_msg);
        // Publish the cropped map
        ref_cropped_map_cloud_->header.frame_id = "map";
        sensor_msgs::PointCloud2 cropped_map_msg;
        pcl::toROSMsg(*ref_cropped_map_cloud_, cropped_map_msg);
        cropped_map_msg.header.stamp = odom_msg->header.stamp;
        cropped_map_msg.header.frame_id = "map";
        map_pub_.publish(cropped_map_msg);
    }
}
