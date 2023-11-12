#pragma once

#include "ECSManager.hpp"
#include "Vector2D.hpp"

void ODE_System_init(Vector2D (*f)(Vector2D));

void ODE_System(ECS_Manager &world, float dt);
