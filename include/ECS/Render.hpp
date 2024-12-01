#pragma once

#include "ECS.hpp"
#include "ECSManager.hpp"

#include <Eigen/Core>

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
Eigen::Vector2d Render_GetMousePosition();

// ---- Util Functions ---- //

// The idea was that the scale functions would just transform the (...)scale_X/Y functions
// would handle the scale factor - esque conversions
//
// The (...)_X/Y functions would handle the transforms. 

// I don't like this and it either needs to change or be made more clear which is which.
double screen2worldscale_X(int screen_x, double screen_width_in_meters);
double screen2worldscale_Y(int screen_y, double screen_height_in_meters);

double screen2world_Y(int screen_y, double screen_height_in_meters);
double screen2world_X(int screen_x, double screen_width_in_meters);

//Scale differences
double world2screenscale_X(double x, double screen_width_in_meters);
double world2screenscale_Y(double y, double screen_height_in_meters);

// Coord transform that assumes orthogonality for the transform
double world2screen_X(double x, double screen_width_in_meters);
double world2screen_Y(double y, double screen_height_in_meters);
