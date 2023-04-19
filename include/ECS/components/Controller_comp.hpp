#pragma once 

#include "Vector2D.hpp"

struct Controller_Component {
    
    int entity_id;

    double K_Gain;
    double B_Gain;

    Vector2D input;
};
