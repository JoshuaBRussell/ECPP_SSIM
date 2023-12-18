#pragma once

#include <Eigen/Core>

struct Acceleration_Component {

    int entity_id;

    Eigen::Vector2f accel; // {x, y}

};
