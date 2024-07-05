#pragma once

#include "ECSManager.hpp"


// This isn't actually needed at this moment in time, but it could serve
// as a useful paradigm to follow in the future for any 'Visualization' related
// systems.
struct constr_visual_config {
        size_t screen_width_in_pixels;
        size_t screen_height_in_pixels;
        double screen_width_in_meters;
        double screen_height_in_meters;
};

void Constraint_Visualization_Init(struct constr_visual_config &constr_visual_config);
void Constraint_Visualization_System(ECS_Manager &world);
