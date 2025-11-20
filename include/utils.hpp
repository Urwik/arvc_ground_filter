#pragma once

#include <iostream>
#include <filesystem>
#include <algorithm>


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

#include "arvc_utils/arvc_utils.hpp"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace fs = std::filesystem;


pcl::PointCloud<pcl::PointXYZL>::Ptr read_cloud( fs::path _path) {
 
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZL>);

    reader.read(_path.string(), *cloud);
    return cloud;
}

template <typename T>
typename pcl::PointCloud<T>::Ptr readPointCloud(const fs::path &path) {
 
    if (!fs::exists(path)) {
        throw std::runtime_error("File does not exist: " + path.string());
    }

    if (path.extension() == ".pcd") {
        pcl::PCDReader reader;
        typename pcl::PointCloud<T>::Ptr cloud (new pcl::PointCloud<T>); 
        reader.read(path.string(), *cloud);
        return cloud;
    }

    if (path.extension() == ".ply") {
        pcl::PLYReader reader;
        typename pcl::PointCloud<T>::Ptr cloud (new pcl::PointCloud<T>); 
        reader.read(path.string(), *cloud);
        return cloud;
    }
}


template <typename T>
void savePointCloud(const typename pcl::PointCloud<T>::Ptr &cloud, const fs::path &path) {
    
    if (!fs::exists(path.parent_path())) {
        fs::create_directories(path.parent_path());        
    }

    if (path.extension() == ".pcd") {
        pcl::PCDWriter writer;
        writer.write(path.string(), *cloud, true);
        return;
    }

    if (path.extension() == ".ply") {
        pcl::PLYWriter writer;
        writer.write(path.string(), *cloud, true);
        return;
    }
}


gt_indices getGroundTruthIndices(pcl::PointCloud<pcl::PointXYZL>::Ptr &_cloud) {
  
    gt_indices indices;
    
    for (size_t i = 0; i < _cloud->points.size(); i++) {

        if (_cloud->points[i].label > 0) {
            indices.truss->push_back(i);
        }
        else 
            indices.ground->push_back(i);
    }
    return indices;
}


// /**
//  * @brief Returns a Voxelized PointCloud
//  * 
//  * @param _cloud_in 
//  * @return pcl::PointCloud<pcl::PointXYZ>::Ptr
//  */
// PointCloud::Ptr voxel_filter( PointCloud::Ptr &_cloud_in ,float leafSize = 0.1)
// {
//   PointCloud::Ptr _cloud_out (new PointCloud);
//   pcl::VoxelGrid<PointT> sor;
//   sor.setInputCloud(_cloud_in);
//   sor.setLeafSize(leafSize, leafSize, leafSize);
//   sor.filter(*_cloud_out);

//   return _cloud_out;
// }


// /**
//  * @brief Filtra la nube de puntos en función de los índices pasados como parámetro
//  * 
//  * @param cloud 
//  * @param optimizeCoefs 
//  * @param distThreshold 
//  * @param maxIterations 
//  * @return pcl::ModelCoefficients::Ptr 
//  */
// pcl::ModelCoefficientsPtr compute_planar_ransac (PointCloud::Ptr &_cloud_in, const bool optimizeCoefs, float distThreshold = 0.03, int maxIterations = 1000)
// {
//   pcl::PointIndices point_indices;
//   pcl::SACSegmentation<PointT> ransac;
//   pcl::ModelCoefficientsPtr plane_coeffs (new pcl::ModelCoefficients);

//   ransac.setInputCloud(_cloud_in);
//   ransac.setOptimizeCoefficients(optimizeCoefs);
//   ransac.setModelType(pcl::SACMODEL_PLANE);
//   ransac.setMethodType(pcl::SAC_RANSAC);
//   ransac.setMaxIterations(maxIterations);
//   ransac.setDistanceThreshold(distThreshold);
//   ransac.segment(point_indices, *plane_coeffs);

//   return plane_coeffs;
// }

// /**
//  * @brief Filtra la nube de puntos en función de los índices pasados como parámetro
//  * 
//  * @param cloud 
//  * @param coefs 
//  * @param distThreshold 
//  * @return pcl::PointIndices::Ptr 
//  */
// pair<pcl::IndicesPtr, pcl::IndicesPtr>
// get_points_near_plane(PointCloud::Ptr &_cloud_in, pcl::ModelCoefficientsPtr &_plane_coeffs, float distThreshold = 0.5f)
// {
//   Eigen::Vector4f coefficients(_plane_coeffs->values.data());
//   pcl::PointXYZ point;
//   pcl::IndicesPtr _plane_inliers (new pcl::Indices);
//   pcl::IndicesPtr _plane_outliers (new pcl::Indices);

//   for (size_t indx = 0; indx < _cloud_in->points.size(); indx++)
//   {
//     point = _cloud_in->points[indx];
//     float distance = pcl::pointToPlaneDistance(point, coefficients);
//     if (pcl::pointToPlaneDistance(point, coefficients) <= distThreshold)
//       _plane_inliers->push_back(indx);
//     else
//       _plane_outliers->push_back(indx);
//   }

//   return pair<pcl::IndicesPtr, pcl::IndicesPtr> {_plane_inliers, _plane_outliers};
// }