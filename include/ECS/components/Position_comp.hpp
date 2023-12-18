#pragma once

#include <Eigen/Core> 

struct Position_Component {
    
    int entity_id;

    Eigen::Vector2f position; // {x, y}

};
