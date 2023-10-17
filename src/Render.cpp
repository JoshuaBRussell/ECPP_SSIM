#include "Render.hpp"

#include <map>

#include "ECS.hpp"
#include "ECSManager.hpp"

#include "Vector2D.hpp"

#include "./components/Motion_comp.hpp"
#include "./components/Render_comp.hpp"
#include "./components/Position_comp.hpp"
#include "./components/Collision_comp.hpp"


#include "raylib-cpp.hpp"
#include "raylib.h"

static std::map<std::string, raylib::Texture2D*> texture_repo;

static int screen_width_in_pixels;
static int screen_height_in_pixels;

static double screen_width_in_meters;
static double screen_height_in_meters;

// ---- Util Functions ---- //

// The idea was that the scale functions would just transform the (...)scale_X/Y functions
// would handle the scale factor - esque conversions
//
// The (...)_X/Y functions would handle the transforms. 
//
// I don't like this and it either needs to change or be made more clear which is which.
static double screen2worldscale_X(int screen_x){
    return screen_x * ((double)screen_width_in_meters/screen_width_in_pixels); 
};

static double screen2worldscale_Y(int screen_y){
    return -(screen_y - screen_height_in_pixels)*(screen_height_in_meters/screen_height_in_pixels);
};

static double screen2world_Y(int screen_y){
    return  -(screen_height_in_meters/(double)screen_height_in_pixels) * (double)(screen_y - screen_height_in_pixels);
};

//Scale differences
static double world2screenscale_X(double x){
    return x * ((double)screen_width_in_pixels/screen_width_in_meters);  
}
static double world2screenscale_Y(double y){
    return y * ((double)screen_height_in_pixels/screen_height_in_meters);
}

// Coord transform that assumes orthogonality for the transform
static double world2screen_X(double x){
    return world2screenscale_X(x);
}
static double world2screen_Y(double y){
    return -world2screenscale_Y(y) + (double)screen_height_in_pixels;
}

Vector2D Input_get_pos_from_mouse(raylib::Mouse &mouse_instance){
    double mouse_x = screen2worldscale_X(mouse_instance.GetPosition().x);
    double mouse_y = screen2world_Y(mouse_instance.GetPosition().y);
    
    return Vector2D(mouse_x, mouse_y);
}

bool Input_is_button_pressed(raylib::Mouse &mouse_instance, int button){
    return mouse_instance.IsButtonReleased(button);
}

// ---- ---- //
void Render_init(struct render_config &render_config){

    screen_width_in_pixels  = render_config.screen_width_in_pixels;
    screen_height_in_pixels = render_config.screen_height_in_pixels;
    screen_width_in_meters  = render_config.screen_width_in_meters;
    screen_height_in_meters = render_config.screen_height_in_meters;
    
    
}

void Render_System_Exclusive(ECS_Manager &world){
   
    BeginDrawing();
    ClearBackground(BLACK); 

    for (auto it = world.get_component_begin<Position_Component>(); 
              it < world.get_component_end<Position_Component>(); it++){
        
        // Check to see if the texture has been seen before
        std::string tex_loc = world.get_component<Render_Component>(it->entity_id)->texture_loc;
        if (texture_repo.find(tex_loc) == texture_repo.end()){
            // Add it to the texture texture_repo
            raylib::Texture2D* texture_ptr = new raylib::Texture2D(tex_loc);
            texture_repo.insert({tex_loc, texture_ptr}); 
        }
        
        Vector2D obj_pos = world.get_component<Position_Component>(it->entity_id)->position;
        double obj_radius = world.get_component<Collision_Component>(it->entity_id)->radius;
        
        double obj_diameter = 2*obj_radius; 
        
        raylib::Texture2D *texture_ptr = texture_repo[tex_loc];
        Vector2 tex_size = texture_ptr->GetSize();
          
        raylib::Rectangle src_rec(0.0, 0.0, tex_size.x, tex_size.y); // Use the entire texture size
        raylib::Rectangle dest_rec(world2screen_X(obj_pos.x), world2screen_Y(obj_pos.y), 
                                   world2screenscale_X(obj_diameter), world2screenscale_Y(obj_diameter));

        //origin is relative to dest_rec
        raylib::Vector2 origin = {world2screenscale_X(obj_diameter)/2, world2screenscale_Y(obj_diameter/2)};
        texture_ptr->Draw(src_rec, dest_rec, origin, 0.0);
            
    }
 
    DrawFPS(10,10);

    EndDrawing();  
    
}

void Render_System_NonExclusive(ECS_Manager &world){
   
    for (auto it = world.get_component_begin<Position_Component>(); 
              it < world.get_component_end<Position_Component>(); it++){
        
        // Check to see if the texture has been seen before
        std::string tex_loc = world.get_component<Render_Component>(it->entity_id)->texture_loc;
        if (texture_repo.find(tex_loc) == texture_repo.end()){
            // Add it to the texture texture_repo
            raylib::Texture2D* texture_ptr = new raylib::Texture2D(tex_loc);
            texture_repo.insert({tex_loc, texture_ptr}); 
        }
        
        Vector2D obj_pos = world.get_component<Position_Component>(it->entity_id)->position;
        double obj_radius = world.get_component<Collision_Component>(it->entity_id)->radius;
        
        double obj_diameter = 2*obj_radius; 
        
        raylib::Texture2D *texture_ptr = texture_repo[tex_loc];
        Vector2 tex_size = texture_ptr->GetSize();
          
        raylib::Rectangle src_rec(0.0, 0.0, tex_size.x, tex_size.y); // Use the entire texture size
        raylib::Rectangle dest_rec(world2screen_X(obj_pos.x), world2screen_Y(obj_pos.y), 
                                   world2screenscale_X(obj_diameter), world2screenscale_Y(obj_diameter));
        
        //origin is relative to dest_rec
        raylib::Vector2 origin = {world2screenscale_X(obj_diameter)/2, world2screenscale_Y(obj_diameter/2)};
        texture_ptr->Draw(src_rec, dest_rec, origin, 0.0);

        //raylib::Text ID_str(std::to_string(it->entity_id), 16.0, raylib::Color(255, 255, 255, 255));
        //ID_str.Draw(world2screen_X(obj_pos.x), world2screen_Y(obj_pos.y));
           
    }
 
    DrawFPS(10,10);

}
