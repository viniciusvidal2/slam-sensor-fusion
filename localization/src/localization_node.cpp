#include "localization/localization_node.h"

LocalizationNode::LocalizationNode() : Node("localization_node")
{
    // Parameters
    this->declare_parameter<bool>("enable_debug", false);
    this->declare_parameter<std::string>("map_data_path", std::string(std::getenv("HOME")) + "/Desktop/map_data");
    this->declare_parameter<std::string>("map_name", "map");
    this->declare_parameter<double>("max_map_optimization_poses", 50.0);

    // Set debug
    debug_ = this->get_parameter("enable_debug").as_bool();

    // Init the map point cloud and transformation with the frames manager
    std::string map_data_path = this->get_parameter("map_data_path").as_string();
    std::string map_name = this->get_parameter("map_name").as_string();
    int maximum_number_of_poses_to_optimize_map_T_global = static_cast<int>(this->get_parameter("max_map_optimization_poses").as_double());
    global_map_frames_manager_ = std::make_shared<GlobalMapFramesManager>(map_data_path, map_name, maximum_number_of_poses_to_optimize_map_T_global);
    map_cloud_ = global_map_frames_manager_->getMapCloud(0.1f);
    applyUniformSubsample(map_cloud_, 3);
    map_T_global_ = global_map_frames_manager_->getMapTGlobal();

    // Init the ICP object to compute Point to Point alignment
    const int num_iterations = 10;
    const float transformation_epsilon = 1e-5f;
    const float max_correspondence_dist = 0.5f; // [m]
    const float mean_accepted_error = 0.05f; // [m]
    icp_ = std::make_shared<ICPPointToPoint>(max_correspondence_dist, num_iterations, mean_accepted_error, transformation_epsilon);
    icp_->setDebugMode(debug_);

    // Init the Stochastic Filter object
    const std::size_t filter_queue_size = 4;
    const float z_score_threshold = 3.0f;
    coarse_pose_filter_ = std::make_shared<StochasticFilter>(filter_queue_size, z_score_threshold);
    fine_pose_filter_ = std::make_shared<StochasticFilter>(filter_queue_size, z_score_threshold);

    // Brute force alignment object to get the initial alignment
    brute_force_alignment_ = std::make_shared<BruteForceAlignment>();
    brute_force_alignment_->setMeanErrorThreshold(0.1f);
    brute_force_alignment_->setXYZStep(0.1f, 0.1f, 0.05f);
    brute_force_alignment_->setXYZRange(1.5f, 1.5f, 0.1f);
    brute_force_alignment_->setRotationStep(M_PI/18.0f);
    brute_force_alignment_->setRotationRange(M_PI/6.0f);

    // Reference transforms
    map_T_sensor_ = Eigen::Matrix4f::Identity();
    odom_T_sensor_previous_ = Eigen::Matrix4f::Identity();
    map_T_ref_ = Eigen::Matrix4f::Identity();

    // Init the cropped map in the ref frame
    ref_cropped_map_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());

    // Create the publishers for the odometry and the point cloud
    map_T_sensor_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization/map_T_sensor", 10);
    map_T_sensor_prior_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization/map_T_sensor_prior", 10);
    odom_T_sensor_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization/odom_T_sensor", 10);
    map_T_sensor_gps_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization/map_T_sensor_gps", 10);
    cropped_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/localization/cropped_scan_map_frame", 10);
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/localization/map", 10);

    // Compass subscriber, will be used to get the yaw angle
    compass_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "/mavros/global_position/compass_hdg",
        10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
            // Invert the yaw based on Ardupilot convention that clockwise is positive
            current_compass_yaw_ = static_cast<float>((90.0 - msg->data) * M_PI / 180.0);
            // Make sure the yaw is in the range -M_PI to M_PI
            if (current_compass_yaw_ > M_PI)
            {
                current_compass_yaw_ -= 2 * M_PI;
            }
            else if (current_compass_yaw_ < -M_PI)
            {
                current_compass_yaw_ += 2 * M_PI;
            }
        });

    // Initialize synchronized subscribers
    pointcloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "/cloud_registered_body");
    gps_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::NavSatFix>>(this, "/mavros/global_position/global");
    odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, "/Odometry");
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(5), *pointcloud_sub_, *gps_sub_, *odom_sub_);
    sync_->registerCallback(std::bind(&LocalizationNode::localizationCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    RCLCPP_INFO(this->get_logger(), "Localization node initialized!");
}

inline void LocalizationNode::computePosePredictionFromOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg,
                                                                Eigen::Matrix4f& odom_T_sensor_current,
                                                                Eigen::Matrix4f& map_T_sensor_current_odom) const
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

    // Calculate the previous_T_current transformation matrix
    const Eigen::Matrix4f previous_T_current(odom_T_sensor_previous_.inverse() * odom_T_sensor_current);

    // Pose prediction in map frame using the relative pose found in odom frame between previous and current frame readings
    map_T_sensor_current_odom = map_T_sensor_ * previous_T_current;
}

const Eigen::Matrix4f LocalizationNode::computeGpsCoarsePoseInMapFrame(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg) const
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

    return map_T_global_.cast<float>() * global_T_sensor;
}

inline nav_msgs::msg::Odometry LocalizationNode::buildNavOdomMsg(const Eigen::Matrix4f& T, 
                                                const std::string& frame_id, 
                                                const std::string& child_frame_id, 
                                                const rclcpp::Time& stamp) const
{
    nav_msgs::msg::Odometry odom_msg;
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

void LocalizationNode::computePoseGainsFromCovarianceMatrices(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg,
                                                              const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg,
                                                              float& odom_gain, float& gps_gain, const bool fixed) const
{
    // If fixed just send constant value to the gains with more value to odometry
    if (fixed)
    {
        odom_gain = 0.95f;
        gps_gain = 0.05f;
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

void LocalizationNode::initializePosesWithFirstReading(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg,
                                                    const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg)
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
}

bool LocalizationNode::performCoarseAlignment(const pcl::PointCloud<PointT>::Ptr& scan_cloud,
                                              const pcl::PointCloud<PointT>::Ptr& map_cloud)
{
    // Perform first alignment if not aligned yet to the start of the trip in map frame
    if (!brute_force_alignment_->firstAlignmentCompleted())
    {
        // Reduce resolution so brute force can run faster
        pcl::PointCloud<PointT>::Ptr map_cloud_temp = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
        *map_cloud_temp = *map_cloud;
        pcl::PointCloud<PointT>::Ptr scan_cloud_temp = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
        *scan_cloud_temp = *scan_cloud;
        applyUniformSubsample(map_cloud_temp, 15);
        removeFloor(map_cloud_temp);
        removeFloor(scan_cloud_temp);

        // Apply the brute force alignment to the cropped scan and the cropped map
        brute_force_alignment_->setInitialGuess(map_T_sensor_);
        brute_force_alignment_->setSourceCloud(scan_cloud_temp);
        brute_force_alignment_->setTargetCloud(map_cloud_temp);
        if (!brute_force_alignment_->alignClouds())
        {
            // If the brute force was not successfull enough, lets run some strong ICP to align
            RCLCPP_WARN(this->get_logger(), "Brute force alignment not successfull, will try ICP.");
            icp_->setTargetPointCloud(map_cloud_temp);
            icp_->setSourcePointCloud(scan_cloud_temp);
            icp_->setInitialTransformation(brute_force_alignment_->getBestTransformation());
            icp_->setMaxCorrespondenceDist(5.0f);
            icp_->setTransformationEpsilon(1e-2f);
            icp_->setAcceptableMeanError(0.4f);
            icp_->setNumIterations(80);
            const auto icp_result = icp_->calculateAlignment();
            if (icp_result.has_converged)
            {
                icp_->setMaxCorrespondenceDist(0.5f);
                icp_->setTransformationEpsilon(1e-5f);
                icp_->setAcceptableMeanError(0.05f);
                icp_->setNumIterations(10);
                brute_force_alignment_->resetFirstAlignment(true);
                coarse_alignment_complete_ = true;
                map_T_sensor_ = icp_result.transformation;
                return true;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "ICP alignment not successfull, will try again next time.");
                brute_force_alignment_->resetFirstAlignment(false);
                return false;
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "First alignment completed, proceeding to fine alignment.");
            coarse_alignment_complete_ = true;
            map_T_sensor_ = brute_force_alignment_->getBestTransformation();
            return true;
        }
    }
    else
    {
        return true;
    }
}

void LocalizationNode::localizationCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud_msg,
                                            const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg,
                                            const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg)
{
    ///////////////////////////////////////// NODE STARTUP /////////////////////////////////////////
    // If the altitude is still wrong, we cannot proceed
    if (gps_msg->altitude < 0)
    {
        if (debug_)
        {
            RCLCPP_WARN(this->get_logger(), "Altitude is still not correct, waiting for a valid GPS message.");
        }
        return;
    }

    if (first_time_)
    {
        initializePosesWithFirstReading(gps_msg, odom_msg);
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
            return;
        }
    }
    
    ///////////////////////////////////////// FINE ALIGNMENT /////////////////////////////////////////
    // Obtain the odometry prediction for the pose both in map and odom frames
    Eigen::Matrix4f odom_T_sensor_current;
    Eigen::Matrix4f map_T_sensor_odom;
    computePosePredictionFromOdometry(odom_msg, odom_T_sensor_current, map_T_sensor_odom);

    // Obtain the coarse pose from GPS and compass in the map frame, based on the global frame information
    const Eigen::Matrix4f map_T_sensor_gps = computeGpsCoarsePoseInMapFrame(gps_msg);

    // Obtain the weighted coarse pose from GPS and Odometry fusion based on covariance
    float gps_compass_gain, odometry_gain;
    computePoseGainsFromCovarianceMatrices(gps_msg, odom_msg, odometry_gain, gps_compass_gain, false);
    Eigen::Matrix4f map_T_sensor_prior = odometry_gain*map_T_sensor_odom + gps_compass_gain*map_T_sensor_gps;
    // Filter out the coarse pose to avoid sudden changes
    coarse_pose_filter_->addPoseToQueue(map_T_sensor_prior);
    map_T_sensor_prior = coarse_pose_filter_->applyGaussianFilterToCurrentPose(map_T_sensor_, map_T_sensor_prior);
    
    // Align the point clouds with ICP to obtain the relative transformation
    icp_->setSourcePointCloud(cropped_scan_cloud);
    icp_->setInitialTransformation(map_T_sensor_prior);
    const auto icp_result = icp_->calculateAlignment();
    map_T_sensor_ = icp_result.transformation;

    // Update the transformation in odom frame
    odom_T_sensor_previous_ = odom_T_sensor_current;

    // Publish the localized pose in map frame
    map_T_sensor_pub_->publish(buildNavOdomMsg(map_T_sensor_, "map", "sensor", pointcloud_msg->header.stamp));

    /////////////////////////////////////////////////////////////////////////////////////////////////////

    if (debug_)
    {
        // Log the time taken to process the callback
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        RCLCPP_INFO(this->get_logger(), "CALLBACK TOOK %f seconds", elapsed.count());
        // Publish debug poses
        map_T_sensor_prior_pub_->publish(buildNavOdomMsg(map_T_sensor_prior, "map", "sensor", pointcloud_msg->header.stamp));
        odom_T_sensor_pub_->publish(buildNavOdomMsg(odom_T_sensor_current, "map", "sensor", pointcloud_msg->header.stamp));
        map_T_sensor_gps_pub_->publish(buildNavOdomMsg(map_T_sensor_gps, "map", "sensor", pointcloud_msg->header.stamp));
        // Publish the cropped scan
        pcl::transformPointCloud(*cropped_scan_cloud, *cropped_scan_cloud, map_T_sensor_);
        cropped_scan_cloud->header.frame_id = "map";
        sensor_msgs::msg::PointCloud2 cropped_scan_msg;
        pcl::toROSMsg(*cropped_scan_cloud, cropped_scan_msg);
        cropped_scan_msg.header = pointcloud_msg->header;
        cropped_scan_msg.header.frame_id = "map";
        cropped_scan_pub_->publish(cropped_scan_msg);
        // Publish the cropped map
        ref_cropped_map_cloud_->header.frame_id = "map";
        sensor_msgs::msg::PointCloud2 cropped_map_msg;
        pcl::toROSMsg(*ref_cropped_map_cloud_, cropped_map_msg);
        cropped_map_msg.header = pointcloud_msg->header;
        cropped_map_msg.header.frame_id = "map";
        map_pub_->publish(cropped_map_msg);
    }
}
