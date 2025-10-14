#ifndef QUADCOPTER_DYNAMICS_H
#define QUADCOPTER_DYNAMICS_H

#include <fstream>
#include <iostream>
#include <math.h>
#include <random>
#include <string>

#include <Eigen/Dense>

#include "quadcopter/parameters.h"

namespace quadcopter
{

class Dynamics
{
public:
    Dynamics(double dt) : dt_(dt) { this->Init(Eigen::VectorXd::Zero(12)); }

    Dynamics(Eigen::VectorXd x0, double dt) : dt_(dt) { this->Init(x0); }

    void Init(Eigen::VectorXd x0);

    // Compute the forces for each motor
    void GetMotorForces(Eigen::VectorXd u);

    // Update the dynamics (discrete)
    void UpdateDynamics(Eigen::VectorXd u0);

    // Allow for parameters to be set by a caller.
    void SetVehicleParams(const VehicleParameters &params);
    void SetEnvironmentParams(const EnvironmentParameters &params);
    VehicleParameters GetVehicleParameters();
    EnvironmentParameters GetEnvironmentParameters();

    // Get the current quadcopter state
    Eigen::VectorXd GetState();

private:
    // States
    Eigen::VectorXd x;
    Eigen::VectorXd x_motors;
    Eigen::VectorXd u_prev;

    // Discrete time update
    double dt_;

    // Disturbances
    std::default_random_engine motor_gen;

    // Parameter management
    VehicleParameters vehicle_params_;
    EnvironmentParameters env_params_;
    std::ofstream quadcopter_param_file;

    // Vehicle parameters
    void SaveParams();
};

} // namespace quadcopter

#endif