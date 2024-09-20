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
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <cilantro/registration/icp_common_instances.hpp>
#include <cilantro/registration/icp_single_transform_point_to_point_metric.hpp>
#include <cilantro/utilities/point_cloud.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include "localization/geo_lib.h"
#include "localization/global_map_frames_manager.h"
#include "localization/icp_point_to_point.h"

using PointT = pcl::PointXYZ;

class LocalizationNode : public rclcpp::Node
{
public:
    LocalizationNode() : Node("localization_node")
    {
        // Init the map point cloud and transformation with the frames manager
        GlobalMapFramesManager global_map_manager("/home/vini/Desktop/map_data", "map", 50);
        map_cloud_ = global_map_manager.getMapCloud(0.2f);
        map_T_global_ = global_map_manager.getMapTGlobal();

        // Init the ICP object to compute Point to Point alignment
        icp_ = std::make_shared<ICPPointToPoint>(5.5f, 50, 1e-5f, 1e-5f);
        icp_->setDebugMode(true);

        // Reference transforms
        map_T_sensor_ = Eigen::Matrix4f::Identity();
        odom_previous_T_sensor_ = Eigen::Matrix4f::Identity();

        // Init the crop box
        min_point_ = Eigen::Vector4f(0., -10.0, -1., 1.0);
        max_point_ = Eigen::Vector4f(12.0, 10.0, 15.0, 1.0);
        crop_box_.setMin(min_point_);
        crop_box_.setMax(max_point_);

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
        pointcloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "/cloud_registered");
        gps_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::NavSatFix>>(this, "/mavros/global_position/global");
        odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, "/Odometry");
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(50), *pointcloud_sub_, *gps_sub_, *odom_sub_);
        sync_->registerCallback(std::bind(&LocalizationNode::localizationCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        RCLCPP_INFO(this->get_logger(), "Localization node initialized!");
    }

private:
    void computePosePredictionFromOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg,
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

    const Eigen::Matrix4f computeGpsCoarsePoseInMapFrame(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg) const
    {
        // Convert the compass yaw to a rotation matrix
        Eigen::Matrix3f R_sensor_gps;
        R_sensor_gps = Eigen::AngleAxisf(current_compass_yaw_, Eigen::Vector3f::UnitZ());
        // Convert the GPS latitude, longitude and altitude to UTM coordinates
        double utm_northing, utm_easting;
        UTM::LLtoUTM(gps_msg->latitude, gps_msg->longitude, utm_northing, utm_easting);
        // Calculate the coarse pose in map frame from the global pose
        Eigen::Matrix4f global_T_sensor(Eigen::Matrix4f::Identity());
        global_T_sensor.block<3, 3>(0, 0) = R_sensor_gps;
        global_T_sensor.block<3, 1>(0, 3) = Eigen::Vector3f(utm_easting, utm_northing, gps_msg->altitude);

        // Print each matrix and the result  
        return map_T_global_.cast<float>() * global_T_sensor;
    }

    void cropTransformMapPointCloudWithRotatedBoundingBox(const Eigen::Matrix4f& map_current_T_sensor_coarse,
                                                          pcl::PointCloud<PointT>::Ptr& cropped_map_cloud)
    {
        // Get the point indices that are inside a radius in the map point cloud
        pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        const float tx = map_current_T_sensor_coarse(0, 3), ty = map_current_T_sensor_coarse(1, 3);
        for (std::size_t i = 0; i < map_cloud_->size(); ++i)
        {
            const float dx = map_cloud_->points[i].x - tx, dy = map_cloud_->points[i].y - ty;
            if (std::sqrt(dx*dx + dy*dy) < map_crop_radius_)
            {
                indices->indices.emplace_back(i);
            }
        }
        // Filter the point cloud to keep only the points inside the radius
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(map_cloud_);
        extract.setIndices(indices);
        extract.setNegative(false);
        extract.filter(*cropped_map_cloud);

        // Transform the map cloud to the current sensor frame
        pcl::transformPointCloud(*cropped_map_cloud, *cropped_map_cloud, map_current_T_sensor_coarse.inverse());
        // Apply the crop box to the map cloud filtered by radius
        crop_box_.setInputCloud(cropped_map_cloud);
        crop_box_.filter(*cropped_map_cloud);
    }

    const nav_msgs::msg::Odometry buildNavOdomMsg(const Eigen::Matrix4f& T, 
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

    const cilantro::PointCloud3f convertPclToCilantro(const pcl::PointCloud<PointT>::Ptr& cloud) const
    {
        cilantro::PointCloud3f cloud_cilantro;
        cloud_cilantro.points.resize(3, cloud->size());
        for (std::size_t i = 0; i < cloud->size(); ++i)
        {
            cloud_cilantro.points(0, i) = cloud->points[i].x;
            cloud_cilantro.points(1, i) = cloud->points[i].y;
            cloud_cilantro.points(2, i) = cloud->points[i].z;
        }

        return cloud_cilantro;
    }

    const Eigen::Matrix4f calculateAlignmentTransformWithICP(pcl::PointCloud<PointT>::Ptr current_scan_cloud,
                                                             pcl::PointCloud<PointT>::Ptr cropped_map_cloud) const
    {
        icp_->setInputPointClouds(current_scan_cloud, cropped_map_cloud);
        icp_->setInitialTransformation(Eigen::Matrix4f::Identity());

        return icp_->calculateAlignmentTransformation();
        // // Convert both clouds to cilantro format
        // cilantro::PointCloud3f scan_cilantro = convertPclToCilantro(current_scan_cloud);
        // cilantro::PointCloud3f map_cilantro = convertPclToCilantro(cropped_map_cloud);

        // // Apply ICP to align the new cloud with the previous one
        // cilantro::SimplePointToPointMetricRigidICP3f icp(scan_cilantro.points, map_cilantro.points);
        // icp.correspondenceSearchEngine().setMaxDistance(5.0f);
        // icp.setConvergenceTolerance(1e-8f).setMaxNumberOfIterations(50);

        // // Log some prints to tell convergence
        // RCLCPP_INFO(this->get_logger(), "Iterations performed: %zu", icp.getNumberOfPerformedIterations());
        // RCLCPP_INFO(this->get_logger(), "Has converged: %d", icp.hasConverged());

        // return icp.hasConverged() ? icp.estimate().getTransform().matrix() : Eigen::Matrix4f::Identity();
    }

    void localizationCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud_msg,
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

        // Get the coarse transformation based on a simple average of the two readings
        const float gps_compass_weight = 0.2f;
        const Eigen::Matrix4f map_current_T_sensor_coarse = (1.0f - gps_compass_weight) * map_current_T_sensor_odom + gps_compass_weight * map_current_T_sensor_gps;

        // Convert and crop the incoming point cloud
        pcl::PointCloud<PointT>::Ptr current_scan_cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*pointcloud_msg, *current_scan_cloud);
        crop_box_.setInputCloud(current_scan_cloud);
        crop_box_.filter(*current_scan_cloud);
        RCLCPP_INFO(this->get_logger(), "Received and filtered point cloud with %zu points", current_scan_cloud->size());

        // Crop the map point cloud with the rotated bounding box surrounding the region
        pcl::PointCloud<PointT>::Ptr cropped_map_cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
        cropTransformMapPointCloudWithRotatedBoundingBox(map_current_T_sensor_coarse, cropped_map_cloud);
        RCLCPP_INFO(this->get_logger(), "Cropped map cloud with %zu points", cropped_map_cloud->size());

        // Align the point clouds with ICP and the this fine tuned transformation
        const Eigen::Matrix4f map_current_T_fine_adjustment = calculateAlignmentTransformWithICP(current_scan_cloud, cropped_map_cloud);

        // Update the map_T_sensor_ transformation
        map_T_sensor_ = map_current_T_fine_adjustment * map_current_T_sensor_coarse;

        // Update the previous odom_T_sensor_ transformation
        odom_previous_T_sensor_ = odom_current_T_sensor;

        // Publish the odometry messages
        map_T_sensor_pub_->publish(buildNavOdomMsg(map_T_sensor_, "map", "sensor", pointcloud_msg->header.stamp));
        map_T_sensor_coarse_pub_->publish(buildNavOdomMsg(map_current_T_sensor_coarse, "map", "sensor", pointcloud_msg->header.stamp));
        odom_T_sensor_pub_->publish(buildNavOdomMsg(odom_current_T_sensor, "map", "sensor", pointcloud_msg->header.stamp));
        map_T_sensor_gps_pub_->publish(buildNavOdomMsg(map_current_T_sensor_gps, "map", "sensor", pointcloud_msg->header.stamp));

        // Log the time taken to process the callback
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        RCLCPP_INFO(this->get_logger(), "Callback took %f seconds", elapsed.count());

        // Publish the cropped scan
        pcl::transformPointCloud(*current_scan_cloud, *current_scan_cloud, map_T_sensor_);
        current_scan_cloud->header.frame_id = "map";
        sensor_msgs::msg::PointCloud2 cropped_scan_msg;
        pcl::toROSMsg(*current_scan_cloud, cropped_scan_msg);
        cropped_scan_msg.header = pointcloud_msg->header;
        cropped_scan_msg.header.frame_id = "map";
        cropped_scan_pub_->publish(cropped_scan_msg);
        // Publish the cropped map
        pcl::transformPointCloud(*cropped_map_cloud, *cropped_map_cloud, map_current_T_sensor_coarse);
        cropped_map_cloud->header.frame_id = "map";
        sensor_msgs::msg::PointCloud2 cropped_map_msg;
        pcl::toROSMsg(*cropped_map_cloud, cropped_map_msg);
        cropped_map_msg.header = pointcloud_msg->header;
        cropped_map_msg.header.frame_id = "map";
        map_pub_->publish(cropped_map_msg);
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

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr map_T_sensor_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr map_T_sensor_coarse_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_T_sensor_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr map_T_sensor_gps_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropped_scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;

    // Yaw angle from compass
    float current_compass_yaw_{0.0}; // -M_PI to M_PI [RAD]

    // Reference transforms
    Eigen::Matrix4f map_T_sensor_;
    Eigen::Matrix4d map_T_global_;
    Eigen::Matrix4f odom_previous_T_sensor_;

    // Point cloud crop box
    Eigen::Vector4f min_point_;
    Eigen::Vector4f max_point_;
    pcl::CropBox<PointT> crop_box_;

    // Map point cloud
    pcl::PointCloud<PointT>::Ptr map_cloud_;

    // Map crop radius
    const float map_crop_radius_{15.0f}; // [m]

    // ICP object
    std::shared_ptr<ICPPointToPoint> icp_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
