#include "localization/global_map_frames_manager.h"

GlobalMapFramesManager::GlobalMapFramesManager(const std::string data_folder, const std::string map_name, const std::size_t num_poses_max)
    : data_folder_(data_folder), map_name_(map_name), num_poses_max_(num_poses_max)
{
}

std::vector<Eigen::Vector3d> GlobalMapFramesManager::loadOdometryPositions(const std::string &odom_positions_file) const
{
    std::vector<Eigen::Vector3d> odom_positions;
    std::ifstream file(odom_positions_file);
    if (!file.is_open())
    {
        std::cerr << "Error opening file " << odom_positions_file << std::endl;
        return odom_positions;
    }

    std::string line;
    while (std::getline(file, line))
    {
        // Skip first line
        if (line == "tx ty tz")
        {
            continue;
        }
        std::istringstream iss(line);
        Eigen::Vector3d pos;
        iss >> pos.x() >> pos.y() >> pos.z();
        odom_positions.push_back(pos);
    }

    return odom_positions;
}

std::vector<std::pair<Eigen::Vector3d, float>> GlobalMapFramesManager::loadGlobalInfo(const std::string &gps_yaw_file)
{
    std::vector<std::pair<Eigen::Vector3d, float>> latlonalt_yaw;
    std::ifstream file(gps_yaw_file);
    if (!file.is_open())
    {
        std::cerr << "Error opening file " << gps_yaw_file << std::endl;
        return latlonalt_yaw;
    }

    std::string line;
    while (std::getline(file, line))
    {
        // Skip first line
        if (line == "lat lon alt y")
        {
            continue;
        }
        std::istringstream iss(line);
        Eigen::Vector3d latlonalt;
        float yaw;
        iss >> latlonalt.x() >> latlonalt.y() >> latlonalt.z() >> yaw;
        latlonalt_yaw.push_back(std::make_pair(latlonalt, yaw));

        // Save to the GPS table
        if (latlonalt.z() > 0)
        {
            gps_altitude_table_.push_back(latlonalt);
        }
    }

    return latlonalt_yaw;
}

float GlobalMapFramesManager::getClosestAltitude(const double lat, const double lon) const
{
    // Check if the table is empty
    if (gps_altitude_table_.empty())
    {
        return 0.0f;
    }

    // Find the closest latitude and longitude and return the correspondent altitude
    double min_dist = std::numeric_limits<double>::max();
    float match_altitude = 0.0f;
    for (auto &latlonalt : gps_altitude_table_)
    {
        const double dist = std::sqrt(std::pow(lat - latlonalt.x(), 2) + std::pow(lon - latlonalt.y(), 2));
        if (dist < min_dist)
        {
            min_dist = dist;
            match_altitude = latlonalt.z();
        }
    }

    return match_altitude;
}

pcl::PointCloud<PointT>::Ptr GlobalMapFramesManager::getMapCloud(const float voxel_size) const
{
    // Check if the map_name_ exists in data_folder_, and if so just load it and return
    // If not, lets merge the scans and save it for next iterations
    std::string map_cloud_path = data_folder_ + "/" + map_name_ + ".pcd";
    if (access(map_cloud_path.c_str(), F_OK) != -1)
    {
        pcl::PointCloud<PointT>::Ptr map_cloud(new pcl::PointCloud<PointT>);
        pcl::io::loadPCDFile<PointT>(map_cloud_path, *map_cloud);
        return map_cloud;
    }
    else
    {
        return mergeScansAndSave(voxel_size);
    }
}

pcl::PointCloud<PointT>::Ptr GlobalMapFramesManager::mergeScansAndSave(const float voxel_size) const
{
    pcl::PointCloud<PointT>::Ptr map_cloud(new pcl::PointCloud<PointT>);
    // Look in the folder for all pcd files, and add them to the map cloud
    DIR *dir;
    struct dirent *ent;

    // Open the directory to look for all pcd files we need to create a map
    if ((dir = opendir(data_folder_.c_str())) != NULL)
    {
        while ((ent = readdir(dir)) != NULL)
        {
            std::string file_name = ent->d_name;
            if (file_name.size() > 4 && file_name.substr(file_name.size() - 4) == ".pcd")
            {
                // Concatenate the root folder with the file name
                std::string full_path = data_folder_ + "/" + file_name;
                // Load the point cloud
                pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
                pcl::io::loadPCDFile<PointT>(full_path, *cloud);
                // Concatenate the clouds
                *map_cloud += *cloud;
            }
        }
        closedir(dir);
    }
    else
    {
        auto error_msg = "Could not open " + data_folder_ + " directory";
        perror(error_msg.c_str());
        return map_cloud;
    }

    if (map_cloud->empty())
    {
        std::cerr << "Error: no point clouds to merge!" << std::endl;
        return map_cloud;
    }

    // Downsample the map cloud
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(map_cloud);
    vg.setLeafSize(voxel_size, voxel_size, voxel_size);
    vg.filter(*map_cloud);
    // Save the map cloud
    pcl::io::savePCDFileBinary(data_folder_ + "/" + map_name_ + ".pcd", *map_cloud);

    return map_cloud;
}

bool GlobalMapFramesManager::filterBadReadings(std::vector<Eigen::Vector3d> &odom_positions,
                                               std::vector<std::pair<Eigen::Vector3d, float>> &latlonalt_yaw) const
{
    // Check if the sizes of the vectors are the same
    if (odom_positions.size() != latlonalt_yaw.size())
    {
        std::cerr << "Error: the sizes of the vectors are not the same!" << std::endl;
        return false;
    }

    // Filter the bad readings
    std::vector<Eigen::Vector3d> odom_positions_filtered;
    std::vector<std::pair<Eigen::Vector3d, float>> latlonalt_yaw_filtered;
    for (size_t i = 0; i < odom_positions.size(); ++i)
    {
        if (latlonalt_yaw[i].first.z() < 0)
        {
            continue;
        }
        if (odom_positions[i].head<2>().norm() > 0.1)
        {
            break;
        }
        odom_positions_filtered.push_back(odom_positions[i]);
        latlonalt_yaw_filtered.push_back(latlonalt_yaw[i]);
    }

    // Update the vectors
    odom_positions = odom_positions_filtered;
    latlonalt_yaw = latlonalt_yaw_filtered;

    return odom_positions.size() > 3;
}

Eigen::Matrix4d GlobalMapFramesManager::getMapTGlobal()
{
    // Load the odometry positions and the global info
    std::string odom_positions_file = data_folder_ + "/odometry_positions.txt";
    std::string gps_yaw_file = data_folder_ + "/gps_imu_poses.txt";
    std::vector<Eigen::Vector3d> odom_positions = loadOdometryPositions(odom_positions_file);
    std::vector<std::pair<Eigen::Vector3d, float>> latlonalt_yaw = loadGlobalInfo(gps_yaw_file);
    filterBadReadings(odom_positions, latlonalt_yaw);
    if (odom_positions.empty() || latlonalt_yaw.empty())
    {
        std::cerr << "Error: no valid odometry or global info data!" << std::endl;
        return Eigen::Matrix4d::Identity();
    }

    // Get the size to compute the best transform as the lowest of the three
    std::vector<std::size_t> sizes{latlonalt_yaw.size(), num_poses_max_};
    const std::size_t compute_size = *std::min_element(sizes.begin(), sizes.end());
    latlonalt_yaw = std::vector<std::pair<Eigen::Vector3d, float>>(latlonalt_yaw.begin(), latlonalt_yaw.begin() + compute_size);

    // Separate the latlonalt and rpy vectors
    std::vector<Eigen::Vector3d> latlonalt;
    std::vector<float> compass_yaw;
    for (size_t i = 0; i < latlonalt_yaw.size(); ++i)
    {
        latlonalt.push_back(latlonalt_yaw[i].first);
        compass_yaw.push_back(latlonalt_yaw[i].second);
    }

    return computeMapTGlobal(latlonalt, compass_yaw);
}

Eigen::Matrix4d GlobalMapFramesManager::computeMapTGlobal(const std::vector<Eigen::Vector3d> &latlonalt,
                                                          const std::vector<float> &compass_yaw) const
{
    // Check if the sizes of the vectors are the same
    if (latlonalt.size() != compass_yaw.size())
    {
        std::cerr << "Error: the sizes of the vectors are not the same!" << std::endl;
        return Eigen::Matrix4d::Identity();
    }

    // Compute mean global translation vector by converting each data to UTM and averaging
    Eigen::Vector3d global_t_map(0.0, 0.0, 0.0);
    for (const auto &llalt : latlonalt)
    {
        double utm_northing, utm_easting;
        UTM::LLtoUTM(llalt.x(), llalt.y(), utm_northing, utm_easting);
        global_t_map += Eigen::Vector3d(utm_easting, utm_northing, llalt.z());
    }
    global_t_map /= latlonalt.size();

    // Compute the average RPY from the compass
    double compass_yaw_avg = 0;
    for (const auto &yaw : compass_yaw)
    {
        compass_yaw_avg += static_cast<double>(yaw);
    }
    compass_yaw_avg /= compass_yaw.size();

    // Compute the global_T_map transformation and invert it for map_T_global
    Eigen::Matrix3d map_R_global = Eigen::AngleAxisd(-compass_yaw_avg, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix4d map_T_global(Eigen::Matrix4d::Identity());
    map_T_global.block<3, 3>(0, 0) = map_R_global;
    map_T_global.block<3, 1>(0, 3) = -map_R_global * global_t_map;

    return map_T_global;
}
