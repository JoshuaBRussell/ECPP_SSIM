#pragma once

#include "ECS.hpp"
#include "ECSManager.hpp"
#include "Vector2D.hpp"

#include <raylib-cpp.hpp>

struct render_config{
    int screen_width_in_pixels;  
    int screen_height_in_pixels;
   
    int screen_width_in_meters;
    int screen_height_in_meters; 
};

Vector2D Input_get_pos_from_mouse(raylib::Mouse &mouse_instance);
bool Input_is_button_pressed(raylib::Mouse &mouse_instance, int button);
bool Input_should_close();

void Render_init(struct render_config &render_config);

void Render_System_Exclusive(ECS_Manager &world);
void Render_System_NonExclusive(ECS_Manager &world);
