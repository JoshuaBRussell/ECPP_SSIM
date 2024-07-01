#pragma once

#include <Eigen/Core>

struct Connector_Component {

    int entity_id;

    Eigen::Vector2d pos; // {x, y} in body space
                         // i.e. relative to body's Center of Mass
   
    // Force/Torque applied at connector location
    Eigen::Vector2d force;
    double torque;

    int attached_entity;

};
