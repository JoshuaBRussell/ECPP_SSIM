#pragma once

#include "ECSManager.hpp"
#include "Vector2D.hpp"

void ODE_System_2D_init(Vector2D (*f)(Vector2D));

void ODE_System_2D(ECS_Manager &world, float dt);
