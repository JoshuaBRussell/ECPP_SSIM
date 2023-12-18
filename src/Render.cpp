#include "Render.hpp"

#include <iomanip>
#include <iostream>
#include <map>

#include "ECS.hpp"
#include "ECSManager.hpp"

#include "./components/Motion_comp.hpp"
#include "./components/Render_comp.hpp"
#include "./components/Position_comp.hpp"
#include "./components/Collision_comp.hpp"
#include "./components/Rotation_comp.hpp"


#include "raylib-cpp.hpp"
#include "raylib.h"

static std::map<std::string, raylib::Texture2D*> texture_repo;

static int screen_width_in_pixels;
static int screen_height_in_pixels;

static double screen_width_in_meters;
static double screen_height_in_meters;



raylib::Vector2 Input_get_pos_from_mouse(raylib::Mouse &mouse_instance){
    double mouse_x = mouse_instance.GetPosition().x;
    double mouse_y = mouse_instance.GetPosition().y;
    
    return raylib::Vector2(mouse_x, mouse_y);
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
/*
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
*/

void Render_System(ECS_Manager &world){
   
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
        raylib::Vector2 origin = {des_height/2, des_width/2};
        float rotation = world.get_component<Rotation_Component>(it->entity_id)->angle;
        texture_ptr->Draw(src_rec, dest_rec, origin, -1*rotation); // Raylib has positive angles going
                                                                   // CW - I prefer the CCW - the way God intended.
        /*
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << rotation;
        std::string mystring = ss.str();
        raylib::Text ID_str(mystring, 16.0, raylib::Color(255, 255, 255, 255));
        ID_str.Draw(x_pos, y_pos);
        */  
    }
 
    DrawFPS(10,10);

}
