//
//  fun.hpp
//  gicp3
//
//  Created by eythymisk on 31/7/24.
//

#ifndef fun_hpp
#define fun_hpp

#include "rot.hpp"
#include "kdTree.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

// The value, the gradient and the Hessian of the objective function
struct ResultSet {
    float value;
    Eigen::VectorXf gradient;
    Eigen::MatrixXf Hessian;
    
    // Default constructor
    ResultSet()
       : value(0.0f), gradient(), Hessian() {}
    
    // Constructor that initializes value, gradient, and Hessian
    ResultSet(float val, const Eigen::VectorXf& grad, const Eigen::MatrixXf& hess)
        : value(val), gradient(grad), Hessian(hess) {}
};

template <typename PointT>
ResultSet get_objective_value_gradient_and_hessian(
    const typename pcl::PointCloud<PointT>::Ptr& fixed,
    const typename pcl::PointCloud<PointT>::Ptr& moving,
    const std::vector<Eigen::Matrix3f>& fixedCovariances,
    const std::vector<Eigen::Matrix3f>& movingCovariances,
    KDTree<PointT>* fixedKDTree,
    const Eigen::VectorXf& parameters)
{
    // Extract translation (first 3 elements) and Euler angles (last 3 elements)
    Eigen::Vector3f t = parameters.head<3>();
    Eigen::Vector3f a = parameters.tail<3>();

    // Get rotation matrix and derivatives from Euler angles
    Eigen::Matrix3f R, dR_dax, dR_day, dR_daz;
    Rot::eul2rotm(a, R, dR_dax, dR_day, dR_daz);

    // Predefined matrices and vectors used inside the loop
    Eigen::Vector3f X, X_, Y, E, dX_dax, dX_day, dX_daz, grad_Q_E, grad_Q_a;
    Eigen::Matrix3f Ax, Ax_, Ay, A, J_E_A, J_E_A_, H_Q_A;
    Eigen::VectorXf grad_Q_p(6);
    Eigen::MatrixXf H_Q_P(6, 6);
    float Q, closestPointDistanceSq;
    size_t closestPointIdx;

    // Initialize variables to accumulate results
    float total_value = 0.0f;
    Eigen::VectorXf total_gradient = Eigen::VectorXf::Zero(6);
    Eigen::MatrixXf total_Hessian = Eigen::MatrixXf::Zero(6, 6);

    // Loop through each point in the moving point cloud
    for (size_t i = 0; i < moving->size(); ++i)
    {
        // Retrieve the i-th point from the moving point cloud and convert it to an Eigen vector
        X = moving->points[i].getVector3fMap();

        // Apply the transformation (rotation and translation) to the moving point
        X_ = R * X + t;

        // Perform nearest neighbor search in the fixed point cloud
        fixedKDTree->knnSearch(X_.data(), 1, &closestPointIdx, &closestPointDistanceSq);

        // Retrieve the matched point from the fixed point cloud and convert it to an Eigen vector
        Y = fixed->points[closestPointIdx].getVector3fMap();

        // Compute the error vector between the transformed moving point and the matched fixed point
        E = X_ - Y;

        // Retrieve covariance matrices for the current moving and fixed points
        Ax = movingCovariances[i];
        Ay = fixedCovariances[closestPointIdx];

        // Transform the covariance matrix of the moving point
        Ax_ = R * Ax * R.transpose();

        // Calculate the error inverse covariance matrix
        A = (Ax_ + Ay).inverse();

        // Compute the derivatives of the position with respect to the Euler angles
        dX_dax = dR_dax * X;
        dX_day = dR_day * X;
        dX_daz = dR_daz * X;

        // Compute the Jacobian of the position with respect to the Euler angles
        J_E_A << dX_dax, dX_day, dX_daz;

        // Calculate the value of the quadratic and its gradient w.r.t. the error vector
        grad_Q_E = A * E;
        Q = E.transpose() * grad_Q_E;

        // Calculate gradient component with respect to the Euler angles
        grad_Q_a = J_E_A.transpose() * grad_Q_E;

        // Combine gradient components into a single vector
        grad_Q_p.head(3) = grad_Q_E;  // Gradient with respect to the translation
        grad_Q_p.tail(3) = grad_Q_a;  // Gradient with respect to the rotation

        // Compute Hessian
        J_E_A_  = A * J_E_A;
        H_Q_A   = J_E_A.transpose() * J_E_A_;

        H_Q_P.topLeftCorner(3, 3)       = A;                    // Hessian block for translation
        H_Q_P.topRightCorner(3, 3)      = J_E_A_;               // Hessian block for translation-rotation cross terms
        H_Q_P.bottomLeftCorner(3, 3)    = J_E_A_.transpose();   // Hessian block for rotation-translation cross terms
        H_Q_P.bottomRightCorner(3, 3)   = H_Q_A;                // Hessian block for rotation

        // Accumulate the value, the gradient, and the Hessian
        total_value += Q;
        total_gradient += grad_Q_p;
        total_Hessian += H_Q_P;
    }

    // Average the values over all points
    total_value /= moving->size();
    total_gradient /= moving->size();
    total_Hessian /= moving->size();

    // Prepare the output
    ResultSet results(total_value, total_gradient, total_Hessian);

    return results;
}

#endif /* fun_hpp */
