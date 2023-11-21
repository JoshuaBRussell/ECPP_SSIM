#pragma once

#include "ECSManager.hpp"
#include "Vector2D.hpp"

void Newtonian_System_init(Vector2D (*f)(Vector2D));

void Newtonian_Constraint_System(ECS_Manager &world, float dt);

