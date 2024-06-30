#pragma once

#include <Eigen/Core>

struct Force_Component {

    int entity_id;

    Eigen::Vector2d force; // {x, y}

};
