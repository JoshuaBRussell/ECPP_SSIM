#pragma once

#include "ECS.hpp"
#include "ECSManager.hpp"

#include <raylib-cpp.hpp>


raylib::Vector2 Input_get_pos_from_mouse(raylib::Mouse &mouse_instance);
bool Input_is_button_pressed(raylib::Mouse &mouse_instance, int button);
bool Input_should_close();

void Render_System_Exclusive(ECS_Manager &world);
void Render_System_NonExclusive(ECS_Manager &world);
void Render_System(ECS_Manager &world);
