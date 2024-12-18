#include "mapping/map_data_save_node.h"

MapDataSaver::MapDataSaver(ros::NodeHandle &nh)
{
    // Parameters
    ros::NodeHandle pnh("~");
    pnh.param("/debug/enable", debug_, false);
    pnh.param("/map_data/save_relative_path", folder_save_path_, static_cast<std::string>("Desktop/map_data"));
    pnh.param("/map_data/map_name", cloud_save_interval_, 10);
    pnh.param("/mapping/cloud_save_interval", cloud_save_interval_, 10);
    pnh.param("/mapping/min_counter_to_account_for_velocity", min_counter_to_account_for_velocity_, 100);
    pnh.param("/mapping/min_velocity_to_count_as_movement", min_velocity_to_count_as_movement_, 0.1f);

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

    // Params to filter out the vehicle in the incoming reading
    float vehicle_length, vehicle_width, vehicle_height;
    pnh.param("/vehicle_region_box/length", vehicle_length, 0.6f);
    pnh.param("/vehicle_region_box/width", vehicle_width, 0.6f);
    pnh.param("/vehicle_region_box/height", vehicle_height, 1.0f);
    vehicle_box_size_ = Eigen::Vector3f(vehicle_length, vehicle_width, vehicle_height);

    // Create a folder, making sure it does not exist before
    // If it exists, delete it and create it again
    folder_save_path_ = std::string(std::getenv("HOME")) + "/" + folder_save_path_;
    if (FileManipulation::directoryExists(folder_save_path_))
    {
        std::string command = "rm -rf " + folder_save_path_ + "/*";
        if (!system(command.c_str()))
        {
            ROS_WARN("Reseting folder %s to store new mapping data!", folder_save_path_.c_str());
        }
    }
    FileManipulation::createDirectory(folder_save_path_);

    // Create the txt file to save the poses received by odometry
    odometry_file_path_ = folder_save_path_ + "/odometry_positions.txt";
    FileManipulation::createTextFile(odometry_file_path_, "tx ty tz\n");

    // Create the txt file to save the GPS data plus the IMU data for poses
    gps_imu_poses_file_path_ = folder_save_path_ + "/gps_imu_poses.txt";
    FileManipulation::createTextFile(gps_imu_poses_file_path_, "lat lon alt y\n");

    // Initialize the point cloud to save the map
    cloud_map_frame_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

    // Compass subscriber will be used to get the yaw angle
    compass_subscription_ = nh.subscribe<std_msgs::Float64>(
        "/mavros/global_position/compass_hdg",
        10,
        &MapDataSaver::compassCallback, this);

    // Initialize synchronized subscribers
    pointcloud_sub_.subscribe(nh, "/lidar_odometry/cloud_registered_body", 10);
    gps_sub_.subscribe(nh, "/mavros/global_position/global", 10);
    odom_sub_.subscribe(nh, "/lidar_odometry/odometry", 10);
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
        SyncPolicy(50), pointcloud_sub_, gps_sub_, odom_sub_));
    sync_->registerCallback(boost::bind(&MapDataSaver::mappingCallback, this, _1, _2, _3));

    // Initialize the last odometry message
    last_odom_ = Eigen::Matrix4f::Identity();
    last_odom_time_ = ros::Time::now();
    current_odom_time_ = ros::Time::now();
}

void MapDataSaver::compassCallback(const std_msgs::Float64::ConstPtr &msg)
{
    // Invert the yaw based on Ardupilot convention that clockwise is positive
    current_compass_yaw_ = (90.0 - msg->data) * M_PI / 180.0; // [RAD]
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

void MapDataSaver::mappingCallback(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg,
                                   const sensor_msgs::NavSatFix::ConstPtr &gps_msg,
                                   const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    // Convert the odometry message to a homogeneous matrix
    Eigen::Matrix4f odom_T_lidar = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf odom_q_lidar(odom_msg->pose.pose.orientation.w,
                                    odom_msg->pose.pose.orientation.x,
                                    odom_msg->pose.pose.orientation.y,
                                    odom_msg->pose.pose.orientation.z);
    Eigen::Vector3f odom_t_lidar(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
    odom_T_lidar.block<3, 3>(0, 0) = odom_q_lidar.toRotationMatrix();
    odom_T_lidar.block<3, 1>(0, 3) = odom_t_lidar;
    // Apply the extrinsics to the sensor frame
    Eigen::Matrix4f odom_T_vehicle = Eigen::Matrix4f::Identity();
    odom_T_vehicle = lidar_T_vehicle_ * odom_T_lidar;

    // Transform and add the point cloud to the map
    pcl::PointCloud<PointT>::Ptr cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*pointcloud_msg, *cloud);
    pcl::transformPointCloud(*cloud, *cloud, odom_T_vehicle);
    filterVehicleBox(*cloud);
    *cloud_map_frame_ += *cloud;
    ++cloud_counter_;

    // Calculate velocity
    const Eigen::Vector3f current_position(odom_T_vehicle.block<3, 1>(0, 3));
    const Eigen::Vector3f previous_position(last_odom_.block<3, 1>(0, 3));
    current_odom_time_ = odom_msg->header.stamp;
    const float dt = (current_odom_time_ - last_odom_time_).toSec();
    const float velocity = (current_position - previous_position).norm() / dt;
    last_odom_ = odom_T_vehicle;
    last_odom_time_ = current_odom_time_;

    // Do not save if conditions are not met
    if (cloud_counter_ > min_counter_to_account_for_velocity_ &&
        velocity < min_velocity_to_count_as_movement_)
    {
        ROS_WARN("Velocity is too low to save the map. Velocity: %f", velocity);
        return;
    }

    // Save the point cloud tile if the counter reaches the save interval
    if (cloud_counter_ % cloud_save_interval_ == 0)
    {
        std::string cloud_file_path = folder_save_path_ + "/cloud_" + std::to_string(cloud_counter_) + ".pcd";
        pcl::io::savePCDFileBinary(cloud_file_path, *cloud_map_frame_);
        if (debug_)
        {
            ROS_INFO("Saved cloud %d", cloud_counter_);
        }
        cloud_map_frame_->clear();
    }

    // Write a line to the odometry file
    // It should be tx ty tz
    std::ofstream odom_positions_file(odometry_file_path_, std::ios::app);
    odom_positions_file << current_position.x() << " "
                        << current_position.y() << " "
                        << current_position.z() << std::endl;
    odom_positions_file.close();

    // Write a line to the GPS IMU file
    // It should be lat lon alt yaw
    std::ofstream gps_imu_poses_file(gps_imu_poses_file_path_, std::ios::app);
    gps_imu_poses_file << std::fixed << std::setprecision(8)
                       << gps_msg->latitude << " "
                       << gps_msg->longitude << " "
                       << gps_msg->altitude << " "
                       << current_compass_yaw_ << std::endl;
    gps_imu_poses_file.close();
}

void MapDataSaver::filterVehicleBox(pcl::PointCloud<PointT> &cloud)
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

void MapDataSaver::onShutdown()
{
    // Save the final point cloud, if we have one left that was not saved
    if (cloud_map_frame_->size() > 0)
    {
        std::string cloud_file_path = folder_save_path_ + "/cloud_" + std::to_string(cloud_counter_) + ".pcd";
        pcl::io::savePCDFileBinary(cloud_file_path, *cloud_map_frame_);
        if (debug_)
        {
            ROS_INFO("Saved cloud %d", cloud_counter_);
        }
    }
}
