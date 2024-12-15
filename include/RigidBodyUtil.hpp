#pragma once

#include <Eigen/Core>

#include "ECSManager.hpp"

// Useful util functions for adding rigid bodies and contraints

void add_rigid_body_to_world(ECS_Manager &world, int entity_id, Eigen::Vector2d pos, double angle);


void add_fixed_pos_constr(ECS_Manager &world, 
                          int entity_id, int rb_id, 
                          Eigen::Vector2d world_pos, Eigen::Vector2d rel_pos);

void add_rel_constr(ECS_Manager &world, 
                     int entity_id, int rb1_id, int rb2_id, 
                     Eigen::Vector2d rel_pos1, Eigen::Vector2d rel_pos2, Eigen::Vector2d init_pos);

void add_complex_shape_to_world(ECS_Manager &world);
