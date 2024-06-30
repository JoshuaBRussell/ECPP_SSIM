#pragma once

#include <Eigen/Core> 

struct Position_Component {
    
    int entity_id;

    Eigen::Vector2d position; // {x, y}

};
