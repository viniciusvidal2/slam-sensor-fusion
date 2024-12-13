#ifndef POINT_CLOUD_PROCESSING_H
#define POINT_CLOUD_PROCESSING_H
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <signal.h>
#include <cstdlib>

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
#include <pcl/search/kdtree.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

using PointT = pcl::PointXYZ;

static inline void cropPointCloudThroughRadius(const Eigen::Matrix4f &T,
                                               const double radius,
                                               pcl::PointCloud<PointT>::Ptr &cloud,
                                               pcl::PointCloud<PointT>::Ptr &cropped_cloud)
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
    kdtree->radiusSearch(center, radius, kdtree_point_indices, kdtree_point_distances);
    indices->indices = kdtree_point_indices;
    // Extract the indices to create the cropped cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*cropped_cloud);
}

static inline void applyUniformSubsample(pcl::PointCloud<PointT>::Ptr &cloud,
                                         const std::size_t point_step)
{
    if (cloud->points.size() < point_step)
    {
        return;
    }

    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    indices->indices.reserve(cloud->size() / point_step);
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

static inline void removeFloor(pcl::PointCloud<PointT>::Ptr &cloud)
{
    // Remove points lower than 0 in Z
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
        if (cloud->points[i].z > 0)
        {
            indices->indices.emplace_back(i);
        }
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*cloud);
}

#endif // POINT_CLOUD_PROCESSING_H
