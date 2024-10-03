//
//  kdTree.hpp
//  gicp3
//
//  Created by eythymisk on 1/8/24.
//

#ifndef kdTree_hpp
#define kdTree_hpp

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "nanoflann.hpp"

// General PointCloudAdaptor
template <typename PointT>
struct PointCloudAdaptor {
    typename pcl::PointCloud<PointT>::Ptr cloud;

    // Constructor
    PointCloudAdaptor(typename pcl::PointCloud<PointT>::Ptr cloud) : cloud(cloud) {}

    // Return the number of points in the cloud
    inline size_t kdtree_get_point_count() const { return cloud->size(); }

    // Return a specific point's coordinate in the specified dimension (0 = x, 1 = y, 2 = z)
    inline float kdtree_get_pt(const size_t idx, int dim) const {
        if (dim == 0)
            return cloud->points[idx].x;
        else if (dim == 1)
            return cloud->points[idx].y;
        else
            return cloud->points[idx].z;
    }

    // Optional: bounding box (not needed here, so return false)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
};

// General KDTree type
template <typename PointT>
using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor<PointT>>,
    PointCloudAdaptor<PointT>,
    3,
    size_t
>;

#endif /* kdTree_hpp */
