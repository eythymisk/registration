//
//  optim.hpp
//  gicp3
//
//  Created by eythymisk on 1/8/24.
//

#ifndef optim_hpp
#define optim_hpp

#include "rot.hpp"
#include <Eigen/Dense>

class Optim {
public:
    // Calculate the optimal step using gradient and Hessian information
    static Eigen::VectorXf calculateStep(
        const Eigen::VectorXf& gradient, // Gradient of the function
        const Eigen::MatrixXf& Hessian   // Hessian matrix of the function
    ) {
        // Solve for the step vector using the Hessian and gradient (more stable than direct inversion)
        Eigen::VectorXf step = Hessian.ldlt().solve(gradient);

        // Return the calculated step
        return step;
    }

    // Check for convergence based on the step size and tolerance
    static bool checkConvergence(
        const Eigen::VectorXf& dp,          // Step
        float TolT,                         // Tolerance for translation
        float TolA                          // Tolerance for rotation
    ) {
        // Extract translation and rotation components
        Eigen::Vector3f dt = dp.head<3>();
        Eigen::Vector3f da = dp.tail<3>();

        // Convert Euler angles to rotation matrix
        Eigen::Matrix3f dR = Rot::eul2rotm(da);

        // Compute the two convergence metrics
        float dt_norm = dt.norm();
        float da_norm = std::acos((dR.trace() - 1) / 2);

        // Check if the convergence conditions are satisfied
        return dt_norm < TolT && da_norm < TolA;
    }

    // Update parameters
    static bool updateParameters(
        Eigen::VectorXf& p,                // Current parameter vector
        const Eigen::VectorXf& gradient,   // Gradient of the function
        const Eigen::MatrixXf& Hessian,    // Hessian matrix of the function
        float TolT = 0.01f,                // Tolerance for translation (default value)
        float TolA = 0.0087f               // Tolerance for rotation (default value)
    ) {
        // Calculate the optimal step using gradient and Hessian information
        Eigen::VectorXf step = calculateStep(gradient, Hessian);

        // Check if the convergence criteria are satisfied
        bool converged = checkConvergence(step, TolT, TolA);

        // Update parameters
        p -= step;

        return converged;
    }
};

#endif /* optim_hpp */
