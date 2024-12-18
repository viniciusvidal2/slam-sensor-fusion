#include "localization/obstacle_point_cloud_generator_node.h"

ObstaclePointCloudGeneratorNode::ObstaclePointCloudGeneratorNode(ros::NodeHandle nh)
{
    // Parameters
    ros::NodeHandle pnh("~");
    pnh.param("/debug/enable", debug_, false);
    pnh.param("/map_data/home_relative_path", relative_folder_path_, static_cast<std::string>("Desktop/map_data"));
    pnh.param("/map_data/name", map_name_, static_cast<std::string>("map"));
    pnh.param("/map_data/voxel_resolution", map_voxel_size_, 0.1f);
    pnh.param("/obstacle_avoidance/scan_crop_radius", scan_crop_radius_, 15.0f);

    // Init the map point cloud with the frames manager
    global_map_frames_manager_ = std::make_shared<GlobalMapFramesManager>(std::string(std::getenv("HOME")) + "/" + relative_folder_path_,
                                                                          map_name_,
                                                                          50);
    map_cloud_ = global_map_frames_manager_->getMapCloud(map_voxel_size_);
    if (map_cloud_->empty())
    {
        ROS_ERROR("Could not get the map point cloud, exiting.");
        return;
    }

    // Initialize the kdtree with the map point cloud
    map_kdtree_ = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>);
    map_kdtree_->setInputCloud(map_cloud_);

    // Create the publishers
    obstacles_lidar_frame_point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/localization/obstacle_ptc_lidar_frame", 10);
    obstacles_map_frame_point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/localization/obstacle_ptc_map_frame", 10);
    map_point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/localization/obstacle_search_map", 10);

    // Initialize synchronized subscribers
    pointcloud_sub_.subscribe(nh, "/lidar_odometry/cloud_registered_body", 3);
    odom_sub_.subscribe(nh, "/localization/map_T_sensor", 3);
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
        SyncPolicy(3), pointcloud_sub_, odom_sub_));
    sync_->registerCallback(boost::bind(&ObstaclePointCloudGeneratorNode::scanCallback, this, _1, _2));

    ROS_INFO("Obstacle point cloud generator node initialized!");
}

void ObstaclePointCloudGeneratorNode::scanCallback(const sensor_msgs::PointCloud2::ConstPtr &scan_msg,
                                                   const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    ///////////////////////////////////////// PREPROCESSING /////////////////////////////////////////
    // Start timer to measure
    auto start = std::chrono::high_resolution_clock::now();

    // Convert the pose message to Eigen matrix
    Eigen::Matrix4f map_T_lidar = Eigen::Matrix4f::Identity();
    map_T_lidar.block<3, 1>(0, 3) = Eigen::Vector3f(pose_msg->pose.pose.position.x,
                                                    pose_msg->pose.pose.position.y,
                                                    pose_msg->pose.pose.position.z);
    map_T_lidar.block<3, 3>(0, 0) = Eigen::Quaternionf(pose_msg->pose.pose.orientation.w,
                                                       pose_msg->pose.pose.orientation.x,
                                                       pose_msg->pose.pose.orientation.y,
                                                       pose_msg->pose.pose.orientation.z)
                                        .toRotationMatrix();

    // Convert the incoming point cloud and subsample
    pcl::PointCloud<PointT>::Ptr scan_cloud_lidar_frame = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*scan_msg, *scan_cloud_lidar_frame);
    applyUniformSubsample(scan_cloud_lidar_frame, 2);

    // Crop the input scan around the sensor frame origin
    pcl::PointCloud<PointT>::Ptr cropped_scan_cloud_lidar_frame = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    cropPointCloudThroughRadius(Eigen::Matrix4f::Identity(), scan_crop_radius_, scan_cloud_lidar_frame, cropped_scan_cloud_lidar_frame);
    if (debug_)
    {
        ROS_INFO("Scan point cloud cropped with %zu points to assess for obstacles.", cropped_scan_cloud_lidar_frame->size());
    }

    // Transform the cropped scan to the map frame
    pcl::PointCloud<PointT>::Ptr scan_cloud_map_frame = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cropped_scan_cloud_lidar_frame, *scan_cloud_map_frame, map_T_lidar);

    ///////////////////////////////////////// OBSTACLES SEARCH /////////////////////////////////////////
    pcl::PointCloud<PointT>::Ptr obstacle_cloud_lidar_frame = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    // Search for obstacles in the map
    for (std::size_t i = 0; i < scan_cloud_map_frame->size(); ++i)
    {
        // Search for the nearest point in the map
        std::vector<int> point_idx(1);
        std::vector<float> point_squared_distance(1);
        map_kdtree_->nearestKSearch(scan_cloud_map_frame->at(i), 1, point_idx, point_squared_distance);

        // Add if the point is an obstacle
        if (point_squared_distance[0] > 3 * map_voxel_size_)
        {
            obstacle_cloud_lidar_frame->push_back(cropped_scan_cloud_lidar_frame->at(i));
        }
    }
    if (obstacle_cloud_lidar_frame->empty())
    {
        if (debug_)
        {
            ROS_WARN("No obstacles found in the current scan.");
        }
        return;
    }

    ///////////////////////////////////////// POINT CLOUD PUBLISHING /////////////////////////////////////////
    // Publish the obstacle point cloud
    sensor_msgs::PointCloud2 obstacle_cloud_msg;
    pcl::toROSMsg(*obstacle_cloud_lidar_frame, obstacle_cloud_msg);
    obstacle_cloud_msg.header.frame_id = "sensor";
    obstacle_cloud_msg.header.stamp = scan_msg->header.stamp;
    obstacles_lidar_frame_point_cloud_pub_.publish(obstacle_cloud_msg);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    if (debug_)
    {
        // Log the time taken to process the callback
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        ROS_INFO("Obstacle generator callback took %f seconds", elapsed.count());
        // Transform the cloud to the map frame and publish
        pcl::PointCloud<PointT>::Ptr obstacle_cloud_map_frame = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
        pcl::transformPointCloud(*obstacle_cloud_lidar_frame, *obstacle_cloud_map_frame, map_T_lidar);
        sensor_msgs::PointCloud2 obstacle_cloud_map_msg;
        pcl::toROSMsg(*obstacle_cloud_map_frame, obstacle_cloud_map_msg);
        obstacle_cloud_map_msg.header.frame_id = "map";
        obstacle_cloud_map_msg.header.stamp = scan_msg->header.stamp;
        obstacles_map_frame_point_cloud_pub_.publish(obstacle_cloud_map_msg);
        // Publish the map point cloud
        sensor_msgs::PointCloud2 map_cloud_msg;
        pcl::toROSMsg(*map_cloud_, map_cloud_msg);
        map_cloud_msg.header.frame_id = "map";
        map_cloud_msg.header.stamp = scan_msg->header.stamp;
        map_point_cloud_pub_.publish(map_cloud_msg);
    }
}
