//
//  main.cpp
//  gicp3
//
//  Created by eythymisk on 30/7/24.
//

#include "../common/kdTree.hpp"
#include "../common/downsampling.hpp"
#include "../common/cov.hpp"
#include "../common/fun.hpp"
#include "../common/rot.hpp"
#include "../common/gicp.hpp"
#include "../common/tforms.hpp"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <chrono>
#include <iostream>
#include <vector>

int K = 20;
float leaf_size = 0.2;
float epsilon = 0.0004f;

int main(int argc, char** argv) {
    /* 1. Load the point clouds from files //////////////////////////////////////////////////////////////// */
    pcl::PointCloud<pcl::PointXYZ>::Ptr fixed(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr moving(new pcl::PointCloud<pcl::PointXYZ>());

    /* Load the fixed point cloud */
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("data/fixed.pcd", *fixed) == -1) {
        PCL_ERROR("Couldn't read fixed.pcd file\n");
        return -1;
    }
    std::cout << "Loaded " << fixed->width * fixed->height << " data points from fixed.pcd." << std::endl;

    /* Load the moving point cloud */
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("data/moving.pcd", *moving) == -1) {
        PCL_ERROR("Couldn't read moving.pcd file\n");
        return -1;
    }
    std::cout << "Loaded " << moving->width * moving->height << " data points from moving.pcd." << std::endl;

    /* 2. Downsample the point clouds using a Voxel Grid Filter //////////////////////////////////////////////////////////////// */
    pcl::PointCloud<pcl::PointXYZ>::Ptr fixedDownsampled(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr movingDownsampled(new pcl::PointCloud<pcl::PointXYZ>());

    /* Downsample the fixed point cloud */
    downsamplePointCloudVoxelGrid<pcl::PointXYZ>(fixed, fixedDownsampled, leaf_size);

    /* Downsample the moving point cloud */
    downsamplePointCloudVoxelGrid<pcl::PointXYZ>(moving, movingDownsampled, leaf_size);
    
    /* 3. Build KD-Trees for Efficient Neighbor Queries */
    /* Build kdtree for the fixed point cloud */
    PointCloudAdaptor<pcl::PointXYZ>* fixedAdaptor = new PointCloudAdaptor<pcl::PointXYZ>(fixedDownsampled);

    KDTree<pcl::PointXYZ>* fixedKDTree = new KDTree<pcl::PointXYZ>(3, *fixedAdaptor, nanoflann::KDTreeSingleIndexAdaptorParams(20));
    fixedKDTree->buildIndex();

    /* Build kdtree for the moving point cloud */
    PointCloudAdaptor<pcl::PointXYZ>* movingAdaptor = new PointCloudAdaptor<pcl::PointXYZ>(movingDownsampled);

    KDTree<pcl::PointXYZ>* movingKDTree = new KDTree<pcl::PointXYZ>(3, *movingAdaptor, nanoflann::KDTreeSingleIndexAdaptorParams(20));
    movingKDTree->buildIndex();

    /* 4. Compute the covariance matrices //////////////////////////////////////////////////////////////// */
    /* Compute the covariance matrices for the fixed point cloud */
    std::vector<Eigen::Matrix3f> fixedCovariances = get_covariance_matrices<pcl::PointXYZ>(fixedDownsampled, fixedKDTree, K , epsilon);

    /* Compute the covariance matrices for the moving point cloud */
    std::vector<Eigen::Matrix3f> movingCovariances = get_covariance_matrices<pcl::PointXYZ>(movingDownsampled, movingKDTree, K,  epsilon);

    /* 5. Estimate the transformation using the Generalized ICP /////////////////////// */
    generalizedICP gicp;
       
    gicp.setVerbose(true);
    rigidtform tform = gicp.estimateTransformation(
        fixedDownsampled, movingDownsampled, fixedCovariances, movingCovariances, fixedKDTree);
    
    std::cout << "Registration completed!" << std::endl;
    std::cout << "Rotation Matrix: \n" << tform.R << std::endl;
    std::cout << "Translation Vector: \n" << tform.t << std::endl;

    /* 6. Use the estimated transformation to register the moving point cloud in the coordinate system of the fixed */
    applyTransformation(moving, moving, tform);
    
    // Optional: Save the transformed point cloud
    if (pcl::io::savePCDFile<pcl::PointXYZ>("data/movingReg.pcd", *moving, true) == -1) {
        PCL_ERROR("Couldn't write movingRegistered.pcd file\n");
        return -1;
    }
    std::cout << "Transformed point cloud saved to 'movingReg.pcd'" << std::endl;

    /* 7. Merge the two point clouds ///////////////////// */
    *fixed += *moving;

    if (pcl::io::savePCDFile<pcl::PointXYZ>("merged.pcd", *fixed, true) == -1) {
        PCL_ERROR("Couldn't write merged.pcd file\n");
        return -1;
    }
    std::cout << "Merged point cloud saved to 'merged.pcd'" << std::endl;

    return 0;
}


    
    


    

    


