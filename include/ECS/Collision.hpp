#pragma once

#include "ECSManager.hpp"
#include "QuadTree.hpp"


void Collision_System(ECS_Manager &world, double dt);

void Collision_System_QT(ECS_Manager &world, double dt, QuadTree &qt);
