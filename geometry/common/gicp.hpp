//
//  gicp.hpp
//  gicp3
//
//  Created by eythymisk on 1/8/24.
//

#ifndef gicp_hpp
#define gicp_hpp

#include "kdTree.hpp"
#include "objective.hpp"
#include "optim.hpp"
#include "rot.hpp"
#include "tforms.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>

class generalizedICP {
public:
    /* Properties with default values */
    rigidtform initTform = rigidtform();  // Initial transformation (defaults to identity)
    int maxIterations = 20;               // Maximum number of iterations
    float TolT = 0.01f;                   // Tolerance for translation
    float TolA = 0.0087f;                 // Tolerance for rotation (Euler angles)
    bool verbose = false;                 // Verbose flag to control printing

    // Constructor with custom values (optional)
    generalizedICP(const rigidtform& initialTform = rigidtform(),
                   int iterations = 20,
                   float tolT = 0.01f,
                   float tolA = 0.0087f,
                   bool verbosity = false)
        : initTform(initialTform), maxIterations(iterations), TolT(tolT), TolA(tolA), verbose(verbosity) {}

    // Optionally, you can add methods to set/get parameters if needed
    void setMaxIterations(int iterations) { maxIterations = iterations; }
    void setTolT(float tolT) { TolT = tolT; }  // Set tolerance for translation
    void setTolA(float tolA) { TolA = tolA; }  // Set tolerance for rotation
    void setVerbose(bool verbosity) { verbose = verbosity; }  // Set verbosity
    void setInitTform(const rigidtform& tform) { initTform = tform; }
    
    /* Methods */
    // Function to print verbose information of the registration process
    void printVerboseInfo(int iter, const Eigen::VectorXf& parameters, float objectiveValue) {
        std::cout << "Iteration " << iter;
        
        std::cout << " | Parameters: ";
        for (int j = 0; j < parameters.size(); ++j) {
            std::cout << std::setw(10) << std::fixed << std::setprecision(6) << parameters(j) << " ";
        }
        
        std::cout << " | Objective value: " << std::setw(10) << std::fixed << std::setprecision(6) << objectiveValue << std::endl;
    }
    
    /* Estimate the transformation between the two point clouds */
    template <typename PointT>
    rigidtform estimateTransformation(
        const typename pcl::PointCloud<PointT>::Ptr& fixed,
        const typename pcl::PointCloud<PointT>::Ptr& moving,
        const std::vector<Eigen::Matrix3f>& fixedCovariances,
        const std::vector<Eigen::Matrix3f>& movingCovariances,
        KDTree<PointT>* fixedKDTree)
    {
        // Get the initial guess for the parameters
        Eigen::VectorXf parameters = tform2p(initTform);
    
        /* Main Loop: Update parameters until convergence or maximum iterations are reached */
        for (int iter = 0; iter < maxIterations; ++iter) {
            // Evaluate the objective function's value, gradient, and Hessian
//            Objective::ResultSet results = Objective::get_objective_value_gradient_and_hessian<PointT>(
//                fixed, moving, fixedCovariances, movingCovariances, fixedKDTree, parameters);
            ResultSet results = get_objective_value_gradient_and_hessian(
                fixed, moving, fixedCovariances, movingCovariances, fixedKDTree, parameters);
            
            // Print the transformation and objective value if verbose is enabled
            if (verbose) {
                printVerboseInfo(iter, parameters, results.value);
            }

            // Update parameters using the gradient and Hessian information
            bool converged = Optim::updateParameters(parameters, results.gradient, results.Hessian, TolT, TolA);
            
            // Check for convergence and exit loop if criteria are met
            if (converged)
                break;
        }
        // Convert the parameter vector back to a rigid tform object
        rigidtform tform = p2tform(parameters);
                
        return tform;
    }
    
};

#endif /* gicp_hpp */
