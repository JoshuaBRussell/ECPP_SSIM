#pragma once

#include <Eigen/Core>

#include "ECSManager.hpp"

void ODE_System_2D_init();

void ODE_System_2D(ECS_Manager &world, double dt);

Eigen::Vector2d ODE_Function(Eigen::Vector2d state_vec, Eigen::Vector2d input){
    return state_vec;
}
