#include "Render.hpp"

#include <map>
#include <Eigen/Core>
#include <assert.h>

#include "ECS.hpp"
#include "ECSManager.hpp"

#include "./components/Render_comp.hpp"
#include "./components/Rotation_comp.hpp"

#include "raylib-cpp.hpp"
#include "raylib.h"

static const ssize_t MAX_PRE_RENDER_SYSTEMS = 256;
static void (*pre_render_systems_array[MAX_PRE_RENDER_SYSTEMS])(ECS_Manager&) = {nullptr};
static ssize_t pre_render_systems_count = 0;

static const ssize_t MAX_POST_RENDER_SYSTEMS = 256;
static void (*post_render_systems_array[MAX_POST_RENDER_SYSTEMS])(ECS_Manager&) = {nullptr};
static ssize_t post_render_systems_count = 0;

static std::map<std::string, raylib::Texture2D*> texture_repo;


static int screen_width_in_pixels;
static int screen_height_in_pixels;

void Render_System_Init(ECS_Manager &world, struct render_config &render_config){
    raylib::Color textColor(LIGHTGRAY);
    InitWindow(render_config.screen_width_in_pixels, render_config.screen_height_in_pixels, render_config.window_title.c_str()); 
    SetTargetFPS(render_config.target_fps);

    screen_width_in_pixels = render_config.screen_width_in_pixels;
    screen_height_in_pixels = render_config.screen_height_in_pixels;

}

void Render_System_Shutdown(){
    CloseWindow();
}

bool Render_System_WindowShouldClose(){
    return WindowShouldClose();
}

void Render_System_add_pre_render(void (*sys)(ECS_Manager&)){
    if (sys == nullptr){
        return; // Do Nothing
    }

    pre_render_systems_array[pre_render_systems_count] = sys;
    pre_render_systems_count++;
}

static void call_pre_render_systems(ECS_Manager &world){

    for (ssize_t i = 0; i < pre_render_systems_count; i++){
        assert(pre_render_systems_array[i] != nullptr);
        (*(pre_render_systems_array[i]))(world);
    }
}

void Render_System_add_post_render(void (*sys)(ECS_Manager&)){
    if (sys == nullptr){
        return; // Do Nothing
    }

    post_render_systems_array[post_render_systems_count] = sys;
    post_render_systems_count++;
}

static void call_post_render_systems(ECS_Manager &world){

    for (ssize_t i = 0; i < post_render_systems_count; i++){
        assert(post_render_systems_array[i] != nullptr);
        (*(post_render_systems_array[i]))(world);
    }
}

void Render_System(ECS_Manager &world){

    call_pre_render_systems(world);
    
    BeginDrawing();
    ClearBackground(BLACK); 

    for (auto it = world.get_component_begin<Render_Component>();
              it < world.get_component_end<Render_Component>(); it++){

        // Check to see if the texture has been seen before
        std::string tex_loc = world.get_component<Render_Component>(it->entity_id)->texture_loc;
        if (texture_repo.find(tex_loc) == texture_repo.end()){
            // Add it to the texture texture_repo
            raylib::Texture2D* texture_ptr = new raylib::Texture2D(tex_loc);
            texture_repo.insert({tex_loc, texture_ptr}); 
        } 

        raylib::Texture2D *texture_ptr = texture_repo[tex_loc];
        Vector2 tex_size = texture_ptr->GetSize();
        
        raylib::Rectangle src_rec(0.0, 0.0, tex_size.x, tex_size.y); // Use the entire texture size
        
        int x_pos = world.get_component<Render_Component>(it->entity_id)->x;
        int y_pos = world.get_component<Render_Component>(it->entity_id)->y; 
        int des_height = world.get_component<Render_Component>(it->entity_id)->height;
        int des_width = world.get_component<Render_Component>(it->entity_id)->width;
        
        raylib::Rectangle dest_rec(x_pos, y_pos, 
                                   des_height, des_width);
        
        //origin is relative to dest_rec
        float x = static_cast<float>(des_height)/2.0;
        float y = static_cast<float>(des_width)/2.0;
        raylib::Vector2 origin = {x, y}; // The intermediate variables are used to avoid a 'narrowing conversion' from 'int' to 'float' error.
        double rotation = (180.0/3.14159)*world.get_component<Rotation_Component>(it->entity_id)->angle;
        texture_ptr->Draw(src_rec, dest_rec, origin, -1*rotation); // Raylib has positive angles going
                                                                   // CW - I prefer the CCW - the way God intended.
        
        DrawText(std::to_string(it->entity_id).c_str(), x_pos, y_pos, 12, GREEN); 

    }

    call_post_render_systems(world);
    
    EndDrawing();

}


// ---- Input System ---- //
// Most of these will be simple wrapper around the current libs functions until (if ever) a different
// input lib is used

bool Render_IsMouseButtonPressed(int button){
    return IsMouseButtonPressed(button);
}

Eigen::Vector2d Render_GetMousePosition(){
    Vector2 val = GetMousePosition(); // Use a intermediary value to avoid a depence on the lower level lib
    // Just return a Eigen vector instead
    return Eigen::Vector2d(val.x, val.y);
}
// ---- Util Functions ---- //

// The idea was that the scale functions would just transform the (...)scale_X/Y functions
// would handle the scale factor - esque conversions
//
// The (...)_X/Y functions would handle the transforms. 

// I don't like this and it either needs to change or be made more clear which is which.
double screen2worldscale_X(int screen_x, double screen_width_in_meters){
    return screen_x * ((double)screen_width_in_meters/screen_width_in_pixels); 
};

double screen2worldscale_Y(int screen_y, double screen_height_in_meters){
    return -(screen_y - screen_height_in_pixels)*((double)screen_height_in_meters/screen_height_in_pixels);
};

double screen2world_Y(int screen_y, double screen_height_in_meters){
    return  -((double)screen_height_in_meters/screen_height_in_pixels) * (screen_y - screen_height_in_pixels) - screen_height_in_meters/2.0;
};

double screen2world_X(int screen_x, double screen_width_in_meters){
    return screen2worldscale_X(screen_x, screen_width_in_meters) - screen_width_in_meters/2.0;
}

//Scale differences
double world2screenscale_X(double x, double screen_width_in_meters){
    return x * ((double)screen_width_in_pixels/screen_width_in_meters) + (double)screen_width_in_pixels/2.0;  
}
double world2screenscale_Y(double y, double screen_height_in_meters){
    return y * ((double)screen_height_in_pixels/screen_height_in_meters) + (double)screen_height_in_pixels/2.0;
}

// Coord transform that assumes orthogonality for the transform
double world2screen_X(double x, double screen_width_in_meters){
    return world2screenscale_X(x, screen_width_in_meters);
}
double world2screen_Y(double y, double screen_height_in_meters){
    return -world2screenscale_Y(y, screen_height_in_meters) + screen_height_in_pixels;
}
