#pragma once

#include "Vector.hpp"
#include "Vector2D.hpp"

enum class INT_METHOD {
    EULER,
    RK4
};

struct ODE_Component {
    
    int entity_id;

    INT_METHOD integration_method;
};
