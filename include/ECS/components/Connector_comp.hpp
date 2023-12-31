#pragma once

#include <Eigen/Core>

struct Connector_Component {

    int entity_id;

    Eigen::Vector2f pos; // {x, y} in body space
                         // i.e. relative to body's Center of Mass
   
    // Force/Torque applied at connector location
    Eigen::Vector2f force;
    Eigen::Vector2f torque;

    int attached_entity;

};
