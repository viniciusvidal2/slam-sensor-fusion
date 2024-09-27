#include "mapping/map_data_save_node.h"

MapDataSaver::MapDataSaver() : Node("map_data_saver")
{
    // Register the shutdown routine
    rclcpp::on_shutdown(std::bind(&MapDataSaver::onShutdown, this));
    
    // Create a folder, making sure it does not exist before
    // If it exists, delete it and create it again
    folder_save_path_ = "/home/vini/Desktop/map_data";
    if (FileManipulation::directoryExists(folder_save_path_))
    {
        std::string command = "rm -rf " + folder_save_path_;
        system(command.c_str());
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
    compass_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "/mavros/global_position/compass_hdg",
        10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
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
        });

    // Initialize synchronized subscribers
    pointcloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "/cloud_registered");
    gps_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::NavSatFix>>(this, "/mavros/global_position/global");
    odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, "/Odometry");
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(50), *pointcloud_sub_, *gps_sub_, *odom_sub_);
    sync_->registerCallback(std::bind(&MapDataSaver::mappingCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void MapDataSaver::mappingCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud_msg,
                const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg,
                const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg)
{
    // Add the point cloud to the map
    pcl::PointCloud<PointT>::Ptr cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*pointcloud_msg, *cloud);
    *cloud_map_frame_ += *cloud;
    ++cloud_counter_;

    // Save the point cloud tile if the counter reaches the save rate
    if (cloud_counter_ % cloud_save_rate_ == 0)
    {
        std::string cloud_file_path = folder_save_path_ + "/cloud_" + std::to_string(cloud_counter_) + ".pcd";
        pcl::io::savePCDFileBinary(cloud_file_path, *cloud_map_frame_);
        RCLCPP_INFO(this->get_logger(), "Saved cloud %d", cloud_counter_);
        cloud_map_frame_->clear();
    }

    // Write a line to the odometry file
    // It should be tx ty tz
    std::ofstream odom_positions_file(odometry_file_path_, std::ios::app);
    odom_positions_file << odom_msg->pose.pose.position.x << " "
                << odom_msg->pose.pose.position.y << " "
                << odom_msg->pose.pose.position.z << std::endl;
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

void MapDataSaver::onShutdown()
{
    // Save the final point cloud, if we have one left that was not saved
    if (cloud_map_frame_->size() > 0)
    {
        std::string cloud_file_path = folder_save_path_ + "/cloud_" + std::to_string(cloud_counter_) + ".pcd";
        pcl::io::savePCDFileBinary(cloud_file_path, *cloud_map_frame_);
        RCLCPP_INFO(this->get_logger(), "Saved cloud %d", cloud_counter_);
    }
}
