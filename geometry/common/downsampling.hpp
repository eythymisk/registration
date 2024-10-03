//
//  downsampling.hpp
//  gicp3
//
//  Created by eythymisk on 7/8/24.
//

#ifndef downsampling_hpp
#define downsampling_hpp

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>

// Template function to downsample a point cloud using Voxel Grid Filter
template <typename PointT>
void downsamplePointCloudVoxelGrid(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                                   typename pcl::PointCloud<PointT>::Ptr& cloud_filtered,
                                   float leaf_size)
{
    // Create the VoxelGrid filter object
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(cloud);

    // Set the voxel grid leaf size (i.e., the size of the voxel grid)
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);

    // Apply the filter
    voxel_grid.filter(*cloud_filtered);
}

// Template function to downsample a point cloud using Random Sampling
template <typename PointT>
void downsamplePointCloudRandomSample(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                                      typename pcl::PointCloud<PointT>::Ptr& cloud_filtered,
                                      int sample_size)
{
    // Create the RandomSample filter object
    pcl::RandomSample<PointT> random_sample;
    random_sample.setInputCloud(cloud);

    // Set the number of samples to keep
    random_sample.setSample(sample_size);

    // Apply the filter
    random_sample.filter(*cloud_filtered);
}

#endif /* downsampling_hpp */
