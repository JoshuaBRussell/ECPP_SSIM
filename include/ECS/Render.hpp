#pragma once

#include "ECS.hpp"
#include "ECSManager.hpp"


// ---- Graphics System ---- //
// Pre-Graphics System
// - Intepretation Systems (e.g. Constraint Visualization)
// 
// Render(ers)
// - Texture Renderer / Draw Calls
//
// Dear ImGui GUI/Plots

struct render_config {
    
    int screen_width_in_pixels;
    int screen_height_in_pixels;
    std::string window_title;
    
    int target_fps;
};

void Render_System_Init(ECS_Manager &world, struct render_config &render_config);
void Render_System_Shutdown(); 

bool Render_System_WindowShouldClose();

// Automatically have Systems called before/after the world rendering
void Render_System_add_pre_render(void (*sys)(ECS_Manager&));
void Render_System_add_post_render(void (*sys)(ECS_Manager&));

void Render_System_Exclusive(ECS_Manager &world);
void Render_System_NonExclusive(ECS_Manager &world);
void Render_System(ECS_Manager &world);

// Let the Render System handle input events since it is easier
typedef enum {
    LEFT_MOUSE_BUTTON    = 0,
    RIGHT_MOUSE_BUTTON   = 1,
    MIDDLE_MOUSE_BUTTON  = 2,
    SIDE_MOUSE_BUTTON    = 3,
    EXTRA_MOUSE_BUTTON   = 4,
    FORWARD_MOUSE_BUTTON = 5,
    BACK_MOUSE_BUTTON    = 6,
} RenderMouseButton;

bool Render_IsMouseButtonPressed(int button);
