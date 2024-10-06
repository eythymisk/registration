//
//  tforms.hpp
//  gicp3
//
//  Created by eythymisk on 7/8/24.
//

#ifndef tforms_hpp
#define tforms_hpp

#include "rot.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <fstream>

struct rigidtform {
    /* properties with default values */
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();  // Rotation Matrix
    Eigen::Vector3f t = Eigen::Vector3f::Zero();      // Translation Vector
    
    Eigen::Matrix4f A = Eigen::Matrix4f::Identity();  // Augmented Matrix

    /* Default constructor */
    rigidtform() = default;
    
    /* Constructor from R and t */
    rigidtform(const Eigen::Matrix3f& rotationMatrix, const Eigen::Vector3f& translationVector) : R(rotationMatrix), t(translationVector) {
        composeAugmentedMatrix();
    }
    
    /* Constructor from euler angles and t */
    rigidtform(const Eigen::Vector3f& eulerAngles, const Eigen::Vector3f& translationVector) : R(Rot::eul2rotm(eulerAngles)), t(translationVector) {
        composeAugmentedMatrix();
    }

    /* Constructor that initializes from augmented matrix A */
    rigidtform(const Eigen::Matrix4f& augmentedMatrix) : A(augmentedMatrix)  {
        decomposeAugmentedMatrix();
    }

    /* Compose the augmented matrix A from R and t */
    void composeAugmentedMatrix() {
        A.block<3, 3>(0, 0) = R;
        A.block<3, 1>(0, 3) = t;
    }
    
    /* Extract R and t from the augmented matrix A */
    void decomposeAugmentedMatrix() {
        R = A.block<3, 3>(0, 0);
        t = A.block<3, 1>(0, 3);
    }
};

// Function to convert a parameter vector to a rigidtform
rigidtform p2tform(const Eigen::VectorXf& parameters) {
    Eigen::Vector3f t = parameters.head<3>();     // Extract translation vector
    Eigen::Vector3f a = parameters.tail<3>();     // Extract Euler angles

    return rigidtform(a, t);                      // Create and return the rigidtform struct
}

// Function to convert a rigidtform to a parameter vector
Eigen::VectorXf tform2p(const rigidtform& tform) {
    Eigen::Vector3f a = Rot::rotm2eul(tform.R);  // Convert rotation matrix to euler angles
    Eigen::Vector3f t = tform.t;                 // Extract translation vector
    
    Eigen::VectorXf parameters(6);
    parameters.head<3>() = t; // Translation
    parameters.tail<3>() = a; // Euler angles
    
    return parameters;
}

/* Apply rigid transformation to a point cloud */
void applyTransformation(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudTransformed,
    const rigidtform& tform
)
{
    // Resize the output point cloud to match the input cloud size
    cloudTransformed->points.resize(cloud->size());
    
    cloudTransformed->width = cloud->size();
    cloudTransformed->height = 1;

    // Extract the rotation matrix and translation vector from the rigidTform object
    Eigen::Matrix3f R = tform.R;
    Eigen::Vector3f t = tform.t;
    
    // Initialize vectors used in the loop
    Eigen::Vector3f X, X_transformed;
    
    // Loop through each point in the input point cloud
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        // Retrieve the i-th point from the input point cloud and convert it to an Eigen vector
        X = cloud->points[i].getVector3fMap();

        // Apply the transformation (rotation and translation) to the point
        X_transformed = R * X + t;

        // Store the transformed point back into the output point cloud
        cloudTransformed->points[i].getVector3fMap() = X_transformed;
    }
}

// Function to print the transformation in a single line
void printTransformation(const rigidtform& tform) {
    Eigen::Matrix3f R = tform.R;
    Eigen::Vector3f t = tform.t;

    std::cout << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " "
              << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " "
              << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << " "
              << t(0) << " " << t(1) << " " << t(2) << "\n";
};

// Function to write poses to a file
int writeTransformationsToFile(const std::string& filename, const std::vector<rigidtform>& tforms) {
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Unable to open file: " << filename << std::endl;
        return -1;
    }

    // Write each pose (rotation matrix and translation vector) in the required format
    for (const auto& tform : tforms) {
        Eigen::Matrix3f R = tform.R;  // Extract the rotation matrix
        Eigen::Vector3f t = tform.t;  // Extract the translation vector
        
        // Write the transformation in the format [R(:,1) R(:,2) R(:,3) t]
        file << R(0,0) << " " << R(1,0) << " " << R(2,0) << " "  // First column of R
                   << R(0,1) << " " << R(1,1) << " " << R(2,1) << " "  // Second column of R
                   << R(0,2) << " " << R(1,2) << " " << R(2,2) << " "  // Third column of R
                   << t(0) << " " << t(1) << " " << t(2) << "\n";      // Translation vector
    }

    // Close the file
    file.close();
    
    return 0;
}

#endif /* tforms_hpp */
