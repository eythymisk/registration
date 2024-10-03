//
//  main.cpp
//  gicp3
//
//  Created by eythymisk on 30/7/24.
//

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <chrono>  // For time measurement
#include <iostream>
#include <vector>
#include "../common/kdTree.hpp"
#include "../common/downsampling.hpp"
#include "../common/cov.hpp"
#include "../common/fun.hpp"
#include "../common/rot.hpp"
#include "../common/gicp.hpp"
#include "../common/tforms.hpp"

int main(int argc, char** argv) {
    /* 1. Load the point clouds from files //////////////////////////////////////////////////////////////// */
    pcl::PointCloud<pcl::PointXYZ>::Ptr fixed(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr moving(new pcl::PointCloud<pcl::PointXYZ>());

    /* Load the fixed point cloud */
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/fixed.pcd", *fixed) == -1) {
        PCL_ERROR("Couldn't read fixed.pcd file\n");
        return -1;
    }
    std::cout << "Loaded " << fixed->width * fixed->height << " data points from fixed.pcd." << std::endl;

    /* Load the moving point cloud */
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/moving.pcd", *moving) == -1) {
        PCL_ERROR("Couldn't read moving.pcd file\n");
        return -1;
    }
    std::cout << "Loaded " << moving->width * moving->height << " data points from moving.pcd." << std::endl;

    /* 2. Downsample the point clouds using a Voxel Grid Filter //////////////////////////////////////////////////////////////// */
    pcl::PointCloud<pcl::PointXYZ>::Ptr fixedDownsampled(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr movingDownsampled(new pcl::PointCloud<pcl::PointXYZ>());

    /* Downsample the fixed point cloud */
    auto start = std::chrono::high_resolution_clock::now();
    
    downsamplePointCloudVoxelGrid<pcl::PointXYZ>(fixed, fixedDownsampled, /*leaf_size*/ 0.1f);
    
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration<double, std::milli>(end - start);
    std::cout << "Fixed point cloud downsampled. New number of fixed data points: " << fixedDownsampled->width << ". Elapsed Time: " << elapsed.count() << " ms" << std::endl;

    /* Downsample the moving point cloud */
    start = std::chrono::high_resolution_clock::now();

    downsamplePointCloudVoxelGrid<pcl::PointXYZ>(moving, movingDownsampled, /*leaf_size*/ 0.1f);

    end = std::chrono::high_resolution_clock::now();
    elapsed = std::chrono::duration<double, std::milli>(end - start);
    std::cout << "Moving point cloud downsampled. New number of moving data points: " << movingDownsampled->width << ". Elapsed Time: " << elapsed.count() << " ms" << std::endl;

    /* 3. Build KD-Trees for Efficient Neighbor Queries */
    /* Build kdtree for the fixed point cloud */
    PointCloudAdaptor<pcl::PointXYZ>* fixedAdaptor = new PointCloudAdaptor<pcl::PointXYZ>(fixedDownsampled);

    start = std::chrono::high_resolution_clock::now();

    KDTree<pcl::PointXYZ>* fixedKDTree = new KDTree<pcl::PointXYZ>(3, *fixedAdaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    fixedKDTree->buildIndex();

    end = std::chrono::high_resolution_clock::now();
    elapsed = std::chrono::duration<double, std::milli>(end - start);
    std::cout << "KDTree for fixed point cloud built. Elapsed time: " << elapsed.count() << " ms." << std::endl;

    /* Build kdtree for the moving point cloud */
    PointCloudAdaptor<pcl::PointXYZ>* movingAdaptor = new PointCloudAdaptor<pcl::PointXYZ>(movingDownsampled);

    start = std::chrono::high_resolution_clock::now();

    KDTree<pcl::PointXYZ>* movingKDTree = new KDTree<pcl::PointXYZ>(3, *movingAdaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    movingKDTree->buildIndex();

    end = std::chrono::high_resolution_clock::now();
    elapsed = std::chrono::duration<double, std::milli>(end - start);
    std::cout << "KDTree for moving point cloud built. Elapsed time: " << elapsed.count() << " ms." << std::endl;
    
    /* 4. Compute the covariance matrices //////////////////////////////////////////////////////////////// */
    /* Compute the covariance matrices for the fixed point cloud */
    start = std::chrono::high_resolution_clock::now();

    std::vector<Eigen::Matrix3f> fixedCovariances = get_covariance_matrices<pcl::PointXYZ>(fixedDownsampled, fixedKDTree, 20 , 0.0004f);

    end = std::chrono::high_resolution_clock::now();
    elapsed = std::chrono::duration<double, std::milli>(end - start);
    std::cout << "Covariance matrices for the fixed point cloud computed. Elapsed Time: " << elapsed.count() << " ms." << std::endl;

    /* Compute the covariance matrices for the moving point cloud */
    start = std::chrono::high_resolution_clock::now();

    std::vector<Eigen::Matrix3f> movingCovariances = get_covariance_matrices<pcl::PointXYZ>(movingDownsampled, movingKDTree, 20, 0.0004f);

    end = std::chrono::high_resolution_clock::now();
    elapsed = std::chrono::duration<double, std::milli>(end - start);
    std::cout << "Covariance matrices for the moving point cloud computed. Elapsed Time: " << elapsed.count() << " ms." << std::endl;
//
//    /* 5. Estimate the transformation using the Generalized ICP */
    generalizedICP gicp;
       
    // Estimate the transformation between the downsampled point clouds
    start = std::chrono::high_resolution_clock::now();
    
    rigidtform tform = gicp.estimateTransformation(
        fixedDownsampled, movingDownsampled, fixedCovariances, movingCovariances, fixedKDTree);
    
    end = std::chrono::high_resolution_clock::now();
    elapsed = std::chrono::duration<double, std::milli>(end - start);
    std::cout << "Transformation estimated. Elapsed Time: " << elapsed.count() << " ms." << std::endl;
       
    std::cout << "Rotation Matrix: \n" << tform.R << std::endl;
    std::cout << "Translation Vector: \n" << tform.t << std::endl;

    /* 6. Use the estimated transformation to register the moving point cloud in the coordinate system of the fixed */
    applyTransformation(moving, moving, tform);
    
    // Optional: Save the transformed point cloud
    if (pcl::io::savePCDFile<pcl::PointXYZ>("../data/movingReg.pcd", *moving, true) == -1) {
        PCL_ERROR("Couldn't write movingRegistered.pcd file\n");
        return -1;
    }
    std::cout << "Transformed point cloud saved to 'data/movingReg.pcd'" << std::endl;

    /* 7. Merge the two point clouds */
    *fixed += *moving;

    if (pcl::io::savePCDFile<pcl::PointXYZ>("../data/merged.pcd", *fixed, true) == -1) {
        PCL_ERROR("Couldn't write merged.pcd file\n");
        return -1;
    }
    std::cout << "Merged point cloud saved to 'data/merged.pcd'" << std::endl;

    return 0;
}


    
    


    

    


