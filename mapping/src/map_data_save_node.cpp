#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <signal.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "mapping/file_manipulation.hpp"

using PointT = pcl::PointXYZ;

class MapDataSaver : public rclcpp::Node
{
public:
    MapDataSaver() : Node("map_data_saver")
    {
        // Register the shutdown routine
        rclcpp::on_shutdown(std::bind(&MapDataSaver::onShutdown, this));
        
        // Create a folder, making sure it does not exist before
        // If it exists, delete it and create it again
        folder_save_path_ = "/home/vini/Desktop/map_data";
        if (directoryExists(folder_save_path_))
        {
            std::string command = "rm -rf " + folder_save_path_;
            system(command.c_str());
        }
        createDirectory(folder_save_path_);

        // Create the txt file to save the poses received by odometry
        odometry_file_path_ = folder_save_path_ + "/odometry_positions.txt";
        createTextFile(odometry_file_path_, "tx ty tz\n");

        // Create the txt file to save the GPS data plus the IMU data for poses
        gps_imu_poses_file_path_ = folder_save_path_ + "/gps_imu_poses.txt";
        createTextFile(gps_imu_poses_file_path_, "lat lon alt y\n");

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
        sync_->registerCallback(std::bind(&MapDataSaver::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud_msg,
                  const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg,
                  const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg)
    {
        // Add the point cloud to the map
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_i = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*pointcloud_msg, *cloud_i);
        pcl::PointCloud<PointT>::Ptr cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
        cloud->points.reserve(cloud_i->points.size());
        for (const auto& point : cloud_i->points)
        {
            PointT pp = PointT(point.x, point.y, point.z);
            cloud->points.emplace_back(pp);
        }
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

    void onShutdown()
    {
        // Save the final point cloud, if we have one left that was not saved
        if (cloud_map_frame_->size() > 0)
        {
            std::string cloud_file_path = folder_save_path_ + "/cloud_" + std::to_string(cloud_counter_) + ".pcd";
            pcl::io::savePCDFileBinary(cloud_file_path, *cloud_map_frame_);
            RCLCPP_INFO(this->get_logger(), "Saved cloud %d", cloud_counter_);
        }
    }

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2,
        sensor_msgs::msg::NavSatFix,
        nav_msgs::msg::Odometry>;

    // Subscribers and synchronizer
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr compass_subscription_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pointcloud_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::NavSatFix>> gps_sub_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;  
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // Paths
    std::string folder_save_path_{};
    std::string odometry_file_path_{};
    std::string gps_imu_poses_file_path_{};

    // Point cloud to save the map in tiles
    int cloud_counter_{0};
    constexpr static int cloud_save_rate_{10};
    pcl::PointCloud<PointT>::Ptr cloud_map_frame_;

    // Yaw angle from compass
    double current_compass_yaw_{0.0}; // -M_PI to M_PI [RAD]
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapDataSaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
