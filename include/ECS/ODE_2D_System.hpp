#pragma once

#include <Eigen/Core>

#include "ECSManager.hpp"

void ODE_System_2D_init();

void ODE_System_2D(ECS_Manager &world, float dt);

Eigen::Vector2f ODE_Function(Eigen::Vector2f state_vec, Eigen::Vector2f input){
    return state_vec;
}
