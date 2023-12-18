#pragma once

#include <Eigen/Dense>

#include "ECSManager.hpp"

void Newtonian_System_init();

void Newtonian_Constraint_System(ECS_Manager &world, float dt);

// Newtonian ODE Function
// state: {pos_x, vel_x, pos_y, vel_y}
// input: {acc_x, acc_y}
Eigen::Vector4f ODE_Function(Eigen::Vector4f state, Eigen::Vector2f input);





