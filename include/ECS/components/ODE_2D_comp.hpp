#pragma once

#include <Eigen/Core>

enum class INT_METHOD {
    EULER,
    RK4
};

struct ODE_2D_Component {
    
    int entity_id;

    INT_METHOD integration_method;
};

Eigen::Vector2d ODE_Function(Eigen::Vector2d, Eigen::Vector2d);
