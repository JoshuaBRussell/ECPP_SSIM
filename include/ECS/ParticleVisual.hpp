#pragma once

#include "ECSManager.hpp"

struct particle_visual_config {
        size_t screen_width_in_pixels;
        size_t screen_height_in_pixels;
        double screen_width_in_meters;
        double screen_height_in_meters;
};

void Particle_Visualization_Init(struct particle_visual_config &particle_visual_config);
void Particle_Visualization_System(ECS_Manager &world);
