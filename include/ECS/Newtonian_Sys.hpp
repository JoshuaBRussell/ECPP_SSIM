#pragma once

#include <Eigen/Dense>

#include "ECSManager.hpp"

void Newtonian_System_init();

void Newtonian_System(ECS_Manager &world, float dt);

// Newtonian ODE Function
// state: {pos_x, vel_x, pos_y, vel_y, theta, theta_dot}
// input: {acc_x, acc_y, theta_ddot}
Eigen::Matrix<float, 6, 1> ODE_Function(Eigen::Matrix<float, 6, 1> state, Eigen::Vector3f input);





