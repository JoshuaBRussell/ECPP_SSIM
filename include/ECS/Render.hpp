#pragma once

#include "ECS.hpp"
#include "ECSManager.hpp"


void Render_System_Exclusive(ECS_Manager &world);
void Render_System_NonExclusive(ECS_Manager &world);
void Render_System(ECS_Manager &world);
