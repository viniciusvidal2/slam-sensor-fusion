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

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> GlobalMapFramesManager::loadGlobalInfoAndRpy(const std::string &gps_rpy_positions_file) const
{
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> latlonalt_rpy;
    std::ifstream file(gps_rpy_positions_file);
    if (!file.is_open())
    {
        std::cerr << "Error opening file " << gps_rpy_positions_file << std::endl;
        return latlonalt_rpy;
    }

    std::string line;
    while (std::getline(file, line))
    {
        // Skip first line
        if (line == "lat lon alt r p y")
        {
            continue;
        }
        std::istringstream iss(line);
        Eigen::Vector3d latlonalt, rpy;
        iss >> latlonalt.x() >> latlonalt.y() >> latlonalt.z() >> rpy.x() >> rpy.y() >> rpy.z();
        latlonalt_rpy.push_back(std::make_pair(latlonalt, rpy));
    }

    return latlonalt_rpy;
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
        perror("Could not open DATA directory");
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

Eigen::Matrix4d GlobalMapFramesManager::getMapTGlobal() const
{
    // If we already have a file with the map_T_global transformation, just load it
    // If not, process the odometry and global info files to compute it
    std::string map_t_global_file = data_folder_ + "/map_T_global.txt";
    std::ifstream file(map_t_global_file);
    if (file.is_open())
    {
        Eigen::Matrix4d map_T_global(Eigen::Matrix4d::Identity());
        for (size_t i = 0; i < 4; ++i)
        {
            for (size_t j = 0; j < 4; ++j)
            {
                file >> map_T_global(i, j);
            }
        }
        return map_T_global;
    }
    else
    {
        // Load the odometry positions and the global info with RPY
        std::string odom_positions_file = data_folder_ + "/odometry_positions.txt";
        std::string gps_rpy_positions_file = data_folder_ + "/gps_imu_poses.txt";
        std::vector<Eigen::Vector3d> odom_positions = loadOdometryPositions(odom_positions_file);
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> latlonalt_rpy = loadGlobalInfoAndRpy(gps_rpy_positions_file);

        // Check how many odometry positions are valid (less than 0.3 meters drift)
        const double max_odom_drift{0.1}; // [m]
        std::vector<Eigen::Vector3d> valid_odom_positions;
        valid_odom_positions.reserve(odom_positions.size());
        for (const auto& odom_pos : odom_positions)
        {
            if (odom_pos.head<2>().norm() < max_odom_drift)
            {
                valid_odom_positions.emplace_back(odom_pos);
            }
            else
            {
                break;
            }
        }

        // Get the size to compute the best transform as the lowest of the three
        std::vector<std::size_t> sizes{valid_odom_positions.size(), latlonalt_rpy.size(), num_poses_max_};
        const std::size_t compute_size = *std::min_element(sizes.begin(), sizes.end());
        valid_odom_positions = std::vector<Eigen::Vector3d>(valid_odom_positions.begin(), valid_odom_positions.begin() + compute_size);
        latlonalt_rpy = std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>(latlonalt_rpy.begin(), latlonalt_rpy.begin() + compute_size);

        // Separate the latlonalt and rpy vectors
        std::vector<Eigen::Vector3d> latlonalt, compass_rpy;
        for (size_t i = 0; i < latlonalt_rpy.size(); ++i)
        {
            latlonalt.push_back(latlonalt_rpy[i].first);
            compass_rpy.push_back(latlonalt_rpy[i].second);
        }

        return computeMapTGlobal(valid_odom_positions, latlonalt, compass_rpy);
    }
}

Eigen::Matrix4d GlobalMapFramesManager::computeMapTGlobal(const std::vector<Eigen::Vector3d> &odom_positions,
                                                          const std::vector<Eigen::Vector3d> &latlonalt,
                                                          const std::vector<Eigen::Vector3d> &compass_rpy) const
{
    // Check if the sizes of the vectors are the same
    if (odom_positions.size() != latlonalt.size() || latlonalt.size() != compass_rpy.size())
    {
        std::cerr << "Error: the sizes of the vectors are not the same!" << std::endl;
        return Eigen::Matrix4d::Identity();
    }

    // Compute mean global translation vector by converting each data to UTM and averaging
    Eigen::Vector3d global_t_map(0.0, 0.0, 0.0);
    for (size_t i = 0; i < latlonalt.size(); ++i)
    {
        double utm_northing, utm_easting;
        UTM::LLtoUTM(latlonalt[i].x(), latlonalt[i].y(), utm_northing, utm_easting);
        global_t_map += Eigen::Vector3d(utm_easting, utm_northing, latlonalt[i].z());
    }
    global_t_map /= latlonalt.size();

    // Compute the average RPY from the compass
    Eigen::Vector3d compass_rpy_avg(0.0, 0.0, 0.0);
    for (size_t j = 0; j < compass_rpy.size(); ++j)
    {
        compass_rpy_avg += compass_rpy[j].cast<double>();
    }
    compass_rpy_avg /= compass_rpy.size();

    // Compute the global_T_map transformation and invert it for map_T_global
    Eigen::Matrix3d global_R_map;
    global_R_map = Eigen::AngleAxisd(compass_rpy_avg.z(), Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(compass_rpy_avg.y(), Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxisd(compass_rpy_avg.x(), Eigen::Vector3d::UnitX());
    Eigen::Matrix4d map_T_global(Eigen::Matrix4d::Identity());
    map_T_global.block<3, 3>(0, 0) = global_R_map.transpose();
    map_T_global.block<3, 1>(0, 3) = -global_R_map.transpose() * global_t_map;

    // Save if to a file
    std::ofstream file(data_folder_ + "/map_T_global.txt");
    if (!file.is_open())
    {
        std::cerr << "Error opening file " << data_folder_ + "/map_T_global.txt" << std::endl;
        return map_T_global;
    }
    for (size_t i = 0; i < 4; ++i)
    {
        for (size_t j = 0; j < 4; ++j)
        {
            file << std::fixed << std::setprecision(6) << map_T_global(i, j) << " ";
        }
    }
    file.close();

    return map_T_global;
}
