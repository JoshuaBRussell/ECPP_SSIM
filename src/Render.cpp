#include "Render.hpp"

#include <iomanip>
#include <iostream>
#include <map>
#include <assert.h>

#include "AudioDevice.hpp"
#include "ECS.hpp"
#include "ECSManager.hpp"

#include "./components/Motion_comp.hpp"
#include "./components/Render_comp.hpp"
#include "./components/Position_comp.hpp"
#include "./components/Collision_comp.hpp"
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
        double rotation = (180.0/3.14159)*world.get_component<Rotation_Component>(it->entity_id)->angle;
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

    call_post_render_systems(world);
}
