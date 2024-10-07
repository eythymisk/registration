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
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <filesystem>  // C++17 and above
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <chrono>

// Reconstruction Parameters

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

// map_grid_step: Voxel size used for the final map resolution in the reconstruction process.
// This parameter defines the granularity of the reconstructed environment.
// Smaller values increase detail and accuracy but may lead to larger output files and longer processing times.
float map_grid_step = 0.2f;

std::vector<std::string> get_pcd_file_names(const std::string& folder_path) {
    std::vector<std::string> pcd_file_names;
    for (const auto& entry : std::filesystem::directory_iterator(folder_path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
            pcd_file_names.push_back(entry.path().string());
        }
    }
    // Sort files alphabetically (if needed)
    std::sort(pcd_file_names.begin(), pcd_file_names.end());
    return pcd_file_names;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Error: Please specify the path to your data." << std::endl;
        std::cerr << "Usage: " << argv[0] << " <folder_path>" << std::endl;
        return -1;
    }

    /* Get folder path from the command-line argument */
    std::string folder_path = argv[1];
    
    /* Get the files contained in the folder */
    std::vector<std::string> pcd_file_names = get_pcd_file_names(folder_path);

    if (pcd_file_names.empty()) {
        std::cerr << "No PCD files found in the specified folder." << std::endl;
        return -1;
    }
        
    /* Vectors to store the relative and the absolute poses */
    std::vector<rigidtform> relative_poses, absolute_poses;
    
    /* Load the first point cloud */
    pcl::PointCloud<pcl::PointXYZ>::Ptr fixed(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_names[0], *fixed) == -1) {
        PCL_ERROR("Couldn't read fixed point cloud file\n");
        return -1;
    }
    std::cout << "Loaded " << fixed->width * fixed->height << " data points from " << pcd_file_names[0] << std::endl;
    
    /* Downsample the point cloud using a Voxel Grid Filter */
    pcl::PointCloud<pcl::PointXYZ>::Ptr fixedDownsampled(new pcl::PointCloud<pcl::PointXYZ>());
    downsamplePointCloudVoxelGrid<pcl::PointXYZ>(fixed, fixedDownsampled, downsample_grid_step);

    /* Build a KDTree for the fixed point cloud */
    PointCloudAdaptor<pcl::PointXYZ>* fixedAdaptor = new PointCloudAdaptor<pcl::PointXYZ>(fixedDownsampled);
    KDTree<pcl::PointXYZ>* fixedKDTree = new KDTree<pcl::PointXYZ>(3, *fixedAdaptor, nanoflann::KDTreeSingleIndexAdaptorParams(20));
    fixedKDTree->buildIndex();

    /* Compute the covariance matrices for the fixed point cloud */
    std::vector<Eigen::Matrix3f> fixedCovariances = get_covariance_matrices<pcl::PointXYZ>(fixedDownsampled, fixedKDTree, K, epsilon);
    
    /* Initialize the transformation and the accumulated transformation as identity */
    rigidtform tform, accumTform;
    
    /* Initialize the map as the first point cloud */
    pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>());
    
    map = fixed;
    
    if (map_grid_step > 0) {
        downsamplePointCloudVoxelGrid<pcl::PointXYZ>(map, map, map_grid_step);
    }
    
    /* Main Loop */
    for (int i = 1; i <  pcd_file_names.size(); ++i) {
        /* Measure the total time for the iteration */
        auto start = std::chrono::high_resolution_clock::now();
        
        /* Load a new point cloud */
        pcl::PointCloud<pcl::PointXYZ>::Ptr moving(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_names[i], *moving) == -1) {
            PCL_ERROR("Couldn't read moving point cloud file\n");
            return -1;
        }
        std::cout << "Loaded " << moving->width * moving->height << " data points from " << pcd_file_names[i] << std::endl;

        /* Downsample the point cloud using a Voxel Grid Filter */
        pcl::PointCloud<pcl::PointXYZ>::Ptr movingDownsampled(new pcl::PointCloud<pcl::PointXYZ>());
        downsamplePointCloudVoxelGrid<pcl::PointXYZ>(moving, movingDownsampled, downsample_grid_step);
        
        /* Build a KDTree for the new point cloud */
        PointCloudAdaptor<pcl::PointXYZ>* movingAdaptor = new PointCloudAdaptor<pcl::PointXYZ>(movingDownsampled);
        KDTree<pcl::PointXYZ>* movingKDTree = new KDTree<pcl::PointXYZ>(3, *movingAdaptor, nanoflann::KDTreeSingleIndexAdaptorParams(20));
        movingKDTree->buildIndex();
        
        /* Compute the covariance matrices for the new point cloud */
        std::vector<Eigen::Matrix3f> movingCovariances = get_covariance_matrices<pcl::PointXYZ>(movingDownsampled, movingKDTree, K, epsilon);
        
        /* Estimate the transformation between the previous and the new point cloud using the Generalized ICP */
        generalizedICP gicp;
        gicp.setInitTform(tform);
        
        tform = gicp.estimateTransformation(fixedDownsampled, movingDownsampled, fixedCovariances, movingCovariances, fixedKDTree);
        
        /* Add the new relative pose in the relative pose sequence */
        relative_poses.push_back(tform);
        
        /* Accumulate the transformation */
        accumTform = rigidtform(accumTform.A * tform.A);
        
        /* Add the new absolute pose in the absolute pose sequence */
        absolute_poses.push_back(accumTform);
        
        /* Transform the new point cloud before integrating to the map */
        pcl::PointCloud<pcl::PointXYZ>::Ptr movingReg(new pcl::PointCloud<pcl::PointXYZ>());
        applyTransformation(moving, movingReg, accumTform);
        
        /* Integrate the new point cloud to the map */
        *map += *movingReg;
        
        if (map_grid_step > 0) {
            downsamplePointCloudVoxelGrid<pcl::PointXYZ>(map, map, map_grid_step);
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration<double, std::milli>(end - start);
        std::cout << "New point cloud integrated to the map. Total Iteration Time: " << elapsed.count() << " ms." << std::endl;
        
        /* Swap the point clouds and covariance matrices (moving becomes the new fixed) */
        fixedDownsampled = movingDownsampled;
        fixedCovariances = movingCovariances;
        fixedAdaptor = movingAdaptor;
        fixedKDTree = movingKDTree;
    }
    
    /* Save the poses to a file */
    if (writeTransformationsToFile("relative_poses.txt", relative_poses) == -1) {
        std::cerr << "Unable to open 'relative_poses.txt'\n" << std::endl;
    }
    std::cout << "Poses saved to 'relative_poses.txt'" << std::endl;
        
    if (writeTransformationsToFile("absolute_poses.txt", absolute_poses) == -1) {
        std::cerr << "Unable to open 'absolute_poses.txt'" << std::endl;
    }
    std::cout << "Poses saved to 'absolute_poses.txt'" << std::endl;

    /* Save the merged point cloud map */
    if (pcl::io::savePCDFile<pcl::PointXYZ>("map.pcd", *map, true) == -1) {
        PCL_ERROR("Couldn't write map.pcd file\n");
        return -1;
    }
    std::cout << "Merged point cloud saved to 'map.pcd'" << std::endl;

    return 0;
}
