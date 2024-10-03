//
//  cov.hpp
//  gicp3
//
//  Created by eythymisk on 30/7/24.
//

#ifndef cov_hpp
#define cov_hpp

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include "kdTree.hpp"

// Function to compute covariance matrices for a point cloud with any point type
template <typename PointT>
std::vector<Eigen::Matrix3f> get_covariance_matrices(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    KDTree<PointT>* kdTree,
    size_t K = 20,
    float epsilon = 0.0004f
)
{
    // Initialize a vector to hold the covariance matrices for each point in the cloud
    std::vector<Eigen::Matrix3f> Covariances(cloud->size());

    // Predefine variables for storing kNN indices and distances
    std::vector<size_t> indicesKNN(K);
    std::vector<float> distancesKNN(K);

    // Predefine variables for computing mean and covariance matrix
    Eigen::Vector3f mean;
    Eigen::Matrix3f Covariance, reconstructedCovariance;
    Eigen::Vector3f X;
    
    // Predefine variables for eigenanalysis of the covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver;
    Eigen::Vector3f eigenvalues;
    Eigen::Matrix3f eigenvectors;
    
    // Create a matrix to hold modified eigenvalues with a small positive value to avoid singularity
    Eigen::Matrix3f modifiedEigenvalues = Eigen::Matrix3f::Identity();
    modifiedEigenvalues(0, 0) = epsilon;
    
    // Iterate over each point in the point cloud
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        // Perform k-nearest neighbors search to find the K closest points to the current point
        kdTree->knnSearch(cloud->points[i].data, K, indicesKNN.data(), distancesKNN.data());

        // Reset accumulators for mean and covariance matrix
        mean = Eigen::Vector3f::Zero();
        Covariance = Eigen::Matrix3f::Zero();
        
        // Accumulate the sum of points and their outer products for covariance calculation
        for (size_t idx : indicesKNN) {
            X = cloud->points[idx].getVector3fMap();  // Efficient Eigen mapping for point coordinates
            
            mean += X;
            Covariance += X * X.transpose();
        }

        // Compute the mean of the points
        mean /= static_cast<float>(K);

        // Compute the covariance matrix by averaging and then subtracting the outer product of the mean
        Covariance /= static_cast<float>(K);
        Covariance -= mean * mean.transpose();
        
        // Perform eigenanalysis on the covariance matrix to obtain eigenvalues and eigenvectors
        eigenSolver.computeDirect(Covariance);
        
        eigenvalues = eigenSolver.eigenvalues();
        eigenvectors = eigenSolver.eigenvectors();
        
        // Construct the modified covariance matrix by incorporating a small positive value in the eigenvalues
        reconstructedCovariance = eigenvectors * modifiedEigenvalues * eigenvectors.transpose();
                
        // Store the reconstructed covariance matrix
        Covariances[i] = reconstructedCovariance;
    }

    return Covariances;
}

#endif /* cov_hpp */
