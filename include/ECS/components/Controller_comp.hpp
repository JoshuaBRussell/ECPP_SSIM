#pragma once 

#include <Eigen/Core>

struct Controller_Component {
    
    int entity_id;

    double K_Gain;
    double B_Gain;

    Eigen::Vector2f input;
};
