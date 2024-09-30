#include "localization/localization_node.h"

LocalizationNode::LocalizationNode() : Node("localization_node")
{
    // Init the map point cloud and transformation with the frames manager
    GlobalMapFramesManager global_map_manager("/home/vini/Desktop/map_data", "map", 50);
    map_cloud_ = global_map_manager.getMapCloud(0.1f);
    applyUniformSubsample(map_cloud_, 2);
    map_T_global_ = global_map_manager.getMapTGlobal();

    // Init the ICP object to compute Point to Point alignment
    const int num_iterations = 15;
    const float transformation_epsilon = 1e-5f;
    const float max_correspondence_dist = 1.0f; // [m]
    const float mean_accepted_error = 0.01f; // [m]
    icp_ = std::make_shared<ICPPointToPoint>(max_correspondence_dist, num_iterations, mean_accepted_error, transformation_epsilon);
    icp_->setDebugMode(false);

    // Init the Stochastic Filter object
    filter_ = std::make_shared<StochasticFilter>();

    // Reference transforms
    map_T_sensor_ = Eigen::Matrix4f::Identity();
    odom_previous_T_sensor_ = Eigen::Matrix4f::Identity();
    map_ref_T_sensor_ = Eigen::Matrix4f::Identity();

    // Init the cropped map in the ref frame
    ref_cropped_map_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());

    // Create the publishers for the odometry and the point cloud
    map_T_sensor_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization/map_T_sensor", 10);
    map_T_sensor_coarse_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization/map_T_sensor_coarse", 10);
    odom_T_sensor_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization/odom_T_sensor", 10);
    map_T_sensor_gps_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization/map_T_sensor_gps", 10);
    cropped_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/localization/cropped_scan_map_frame", 10);
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/localization/map", 10);

    // Compass subscriber will be used to get the yaw angle
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
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(50), *pointcloud_sub_, *gps_sub_, *odom_sub_);
    sync_->registerCallback(std::bind(&LocalizationNode::localizationCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    RCLCPP_INFO(this->get_logger(), "Localization node initialized!");
}

inline void LocalizationNode::computePosePredictionFromOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg,
                                        Eigen::Matrix4f& odom_current_T_sensor,
                                        Eigen::Matrix4f& map_current_T_sensor_odom) const
{
    // Get the current pose of the sensor in the odometry frame
    Eigen::Quaternionf odom_current_q_sensor(odom_msg->pose.pose.orientation.w,
                                                odom_msg->pose.pose.orientation.x,
                                                odom_msg->pose.pose.orientation.y,
                                                odom_msg->pose.pose.orientation.z);
    Eigen::Vector3f odom_current_t_sensor(odom_msg->pose.pose.position.x,
                                            odom_msg->pose.pose.position.y,
                                            odom_msg->pose.pose.position.z);
    odom_current_T_sensor.setIdentity();
    odom_current_T_sensor.block<3, 3>(0, 0) = odom_current_q_sensor.toRotationMatrix();
    odom_current_T_sensor.block<3, 1>(0, 3) = odom_current_t_sensor;

    // Calculate the odom_current_T_odom_previous transformation matrix
    const Eigen::Matrix4f odom_current_T_odom_previous(odom_current_T_sensor * odom_previous_T_sensor_.inverse());

    // Pose prediction in map frame using the relative pose found in odom frame between previous and current frame readings
    map_current_T_sensor_odom = odom_current_T_odom_previous * map_T_sensor_;
}

const Eigen::Matrix4f LocalizationNode::computeGpsCoarsePoseInMapFrame(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg) const
{
    // Convert the compass yaw to a rotation matrix
    Eigen::Matrix3f R_sensor_gps;
    R_sensor_gps = Eigen::AngleAxisf(current_compass_yaw_, Eigen::Vector3f::UnitZ());
    // Convert the GPS latitude, longitude and altitude to UTM coordinates
    double utm_northing, utm_easting;
    UTM::LLtoUTM(gps_msg->latitude, gps_msg->longitude, utm_northing, utm_easting);
    // Calculate the pose in map frame from the pose in global frame
    Eigen::Matrix4f global_T_sensor(Eigen::Matrix4f::Identity());
    global_T_sensor.block<3, 3>(0, 0) = R_sensor_gps;
    global_T_sensor.block<3, 1>(0, 3) = Eigen::Vector3f(utm_easting, utm_northing, gps_msg->altitude);

    return map_T_global_.cast<float>() * global_T_sensor;
}

void LocalizationNode::cropPointCloudThroughRadius(const Eigen::Matrix4f& T,
                                    const pcl::PointCloud<PointT>::Ptr& cloud,
                                    pcl::PointCloud<PointT>::Ptr& cropped_cloud) const
{
    // Initialize a kdtree with the cloud
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
    kdtree->setInputCloud(cloud);
    // Get the point indices inside the radius
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    indices->indices.reserve(cloud->size());
    const PointT center = PointT(T(0, 3), T(1, 3), T(2, 3));
    std::vector<int> kdtree_point_indices;
    std::vector<float> kdtree_point_distances;
    kdtree->radiusSearch(center, cloud_crop_radius_, kdtree_point_indices, kdtree_point_distances);
    indices->indices = kdtree_point_indices;
    // Extract the indices to create the cropped cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*cropped_cloud);
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

inline void LocalizationNode::applyUniformSubsample(pcl::PointCloud<PointT>::Ptr& cloud, const std::size_t point_step) const
{
    if (cloud->points.size() < point_step)
    {
        return;
    }

    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    indices->indices.reserve(cloud->size()/point_step);
    for (std::size_t i = 0; i < cloud->size(); i += point_step)
    {
        indices->indices.emplace_back(i);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*cloud);
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

void LocalizationNode::localizationCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud_msg,
                            const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg,
                            const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg)
{
    // Start timer to measure
    auto start = std::chrono::high_resolution_clock::now();
    
    // Obtain the odometry prediction for the pose both in map and odom frames
    Eigen::Matrix4f odom_current_T_sensor;
    Eigen::Matrix4f map_current_T_sensor_odom;
    computePosePredictionFromOdometry(odom_msg, odom_current_T_sensor, map_current_T_sensor_odom);

    // Obtain the coarse pose from GPS and compass in the map frame, based on the global frame information
    const Eigen::Matrix4f map_current_T_sensor_gps = computeGpsCoarsePoseInMapFrame(gps_msg);

    // Add both poses to filter and obtain the weighted coarse pose from GPS and Odometry fusion
    float gps_compass_gain, odometry_gain;
    computePoseGainsFromCovarianceMatrices(gps_msg, odom_msg, odometry_gain, gps_compass_gain, false);
    const Eigen::Matrix4f map_current_T_sensor_coarse = odometry_gain*map_current_T_sensor_odom + gps_compass_gain*map_current_T_sensor_gps;

    // Convert the incoming point cloud and subsample
    pcl::PointCloud<PointT>::Ptr scan_cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*pointcloud_msg, *scan_cloud);
    applyUniformSubsample(scan_cloud, 2);

    // Crop the input scan around the sensor frame origin
    pcl::PointCloud<PointT>::Ptr cropped_scan_cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    cropPointCloudThroughRadius(Eigen::Matrix4f::Identity(), scan_cloud, cropped_scan_cloud);
    // Crop the map point cloud around the coarse sensor position in map frame
    // Only do it if we have a new reference frame measured by walked distance
    const Eigen::Matrix4f map_ref_T_map_current = map_ref_T_sensor_ * map_current_T_sensor_coarse.inverse();
    if (map_ref_T_map_current.block<3, 1>(0, 3).norm() > ref_frame_distance_ || ref_cropped_map_cloud_->empty())
    {
        cropPointCloudThroughRadius(map_current_T_sensor_coarse, map_cloud_, ref_cropped_map_cloud_);
        map_ref_T_sensor_ = map_current_T_sensor_coarse;
    }
    
    // Align the point clouds with ICP to obtain the relative transformation
    icp_->setInputPointClouds(cropped_scan_cloud, ref_cropped_map_cloud_);
    icp_->setInitialTransformation(map_current_T_sensor_coarse);
    map_T_sensor_ = icp_->calculateAlignmentTransformation();

    // Update the transformation in odom frame
    odom_previous_T_sensor_ = odom_current_T_sensor;

    // Publish the odometry messages
    map_T_sensor_pub_->publish(buildNavOdomMsg(map_T_sensor_, "map", "sensor", pointcloud_msg->header.stamp));
    map_T_sensor_coarse_pub_->publish(buildNavOdomMsg(map_current_T_sensor_coarse, "map", "sensor", pointcloud_msg->header.stamp));
    odom_T_sensor_pub_->publish(buildNavOdomMsg(odom_current_T_sensor, "map", "sensor", pointcloud_msg->header.stamp));
    map_T_sensor_gps_pub_->publish(buildNavOdomMsg(map_current_T_sensor_gps, "map", "sensor", pointcloud_msg->header.stamp));

    // Log the time taken to process the callback
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    RCLCPP_INFO(this->get_logger(), "CALLBACK TOOK %f seconds", elapsed.count());

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
