#pragma once

#include "ECS.hpp"
#include "ECSManager.hpp"

// Automatically have Systems called before/after the world rendering
void Render_System_add_pre_render(void (*sys)(ECS_Manager&));
void Render_System_add_post_render(void (*sys)(ECS_Manager&));

void Render_System_Exclusive(ECS_Manager &world);
void Render_System_NonExclusive(ECS_Manager &world);
void Render_System(ECS_Manager &world);
