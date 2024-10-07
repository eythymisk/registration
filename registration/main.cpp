//
//  main.cpp
//  gicp3
//
//  Created by eythymisk on 30/7/24.
//

#include "../common/kdTree.hpp"
#include "../common/downsampling.hpp"
#include "../common/cov.hpp"
#include "../common/objective.hpp"
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

// Registration Parameters

// downsample_grid_step: Voxel size used in voxel grid downsampling during preprocessing.
// Smaller values retain more points but increase computational cost.
// Larger values reduce the number of points, speeding up the process but may sacrifice precision.
float downsample_grid_step = 0.2;

// K: Number of nearest neighbors (k-NN) for analyzing each point's neighborhood.
// This is used to determine local geometric features.
// Higher values provide more neighborhood information but may slow down processing.
int K = 20;

// epsilon: Anisotropic error adjustment factor for the registration process.
// This parameter controls the level of anisotropy in the error, allowing for more error
// along the normal direction of the points. A smaller value increases
// precision, but may lead to numerical issues.
float epsilon = 0.0004f;

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Error: You need to specify the file names for the fixed and moving point clouds." << std::endl;
        std::cerr << "Usage: " << argv[0] << " <fixed.pcd> <moving.pcd>" << std::endl;
        return -1;
    }

    std::string fixedFile = argv[1];
    std::string movingFile = argv[2];

    /* 1. Load the point clouds from files */
    pcl::PointCloud<pcl::PointXYZ>::Ptr fixed(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr moving(new pcl::PointCloud<pcl::PointXYZ>());

    /* Load the fixed point cloud */
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(fixedFile, *fixed) == -1) {
        PCL_ERROR("Couldn't read file: %s\n", fixedFile.c_str());
        return -1;
    }
    std::cout << "Loaded " << fixed->width * fixed->height << " data points from " << fixedFile << "." << std::endl;

    /* Load the moving point cloud */
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(movingFile, *moving) == -1) {
        PCL_ERROR("Couldn't read file: %s\n", movingFile.c_str());
        return -1;
    }
    std::cout << "Loaded " << moving->width * moving->height << " data points from " << movingFile << "." << std::endl;

    /* 2. Downsample the point clouds using a Voxel Grid Filter */
    pcl::PointCloud<pcl::PointXYZ>::Ptr fixedDownsampled(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr movingDownsampled(new pcl::PointCloud<pcl::PointXYZ>());

    /* Downsample the fixed point cloud */
    downsamplePointCloudVoxelGrid<pcl::PointXYZ>(fixed, fixedDownsampled, downsample_grid_step);

    /* Downsample the moving point cloud */
    downsamplePointCloudVoxelGrid<pcl::PointXYZ>(moving, movingDownsampled, downsample_grid_step);
    
    /* 3. Build KD-Trees for Efficient Neighbor Queries */
    /* Build kdtree for the fixed point cloud */
    PointCloudAdaptor<pcl::PointXYZ>* fixedAdaptor = new PointCloudAdaptor<pcl::PointXYZ>(fixedDownsampled);

    KDTree<pcl::PointXYZ>* fixedKDTree = new KDTree<pcl::PointXYZ>(3, *fixedAdaptor, nanoflann::KDTreeSingleIndexAdaptorParams(20));
    fixedKDTree->buildIndex();

    /* Build kdtree for the moving point cloud */
    PointCloudAdaptor<pcl::PointXYZ>* movingAdaptor = new PointCloudAdaptor<pcl::PointXYZ>(movingDownsampled);

    KDTree<pcl::PointXYZ>* movingKDTree = new KDTree<pcl::PointXYZ>(3, *movingAdaptor, nanoflann::KDTreeSingleIndexAdaptorParams(20));
    movingKDTree->buildIndex();

    /* 4. Compute the covariance matrices */
    /* Compute the covariance matrices for the fixed point cloud */
    std::vector<Eigen::Matrix3f> fixedCovariances = get_covariance_matrices<pcl::PointXYZ>(fixedDownsampled, fixedKDTree, K , epsilon);

    /* Compute the covariance matrices for the moving point cloud */
    std::vector<Eigen::Matrix3f> movingCovariances = get_covariance_matrices<pcl::PointXYZ>(movingDownsampled, movingKDTree, K,  epsilon);

    /* 5. Estimate the transformation using the Generalized ICP  */
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
    if (pcl::io::savePCDFile<pcl::PointXYZ>("movingReg.pcd", *moving, true) == -1) {
        PCL_ERROR("Couldn't write movingRegistered.pcd file\n");
        return -1;
    }
    std::cout << "Transformed point cloud saved to 'movingReg.pcd'" << std::endl;

    /* 7. Merge the two point clouds */
    *fixed += *moving;

    if (pcl::io::savePCDFile<pcl::PointXYZ>("merged.pcd", *fixed, true) == -1) {
        PCL_ERROR("Couldn't write merged.pcd file\n");
        return -1;
    }
    std::cout << "Merged point cloud saved to 'merged.pcd'" << std::endl;

    return 0;
}


    
    


    

    


