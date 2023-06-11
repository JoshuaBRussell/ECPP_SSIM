#include "Render.hpp"

#include <map>

#include "ECS.hpp"
#include "ECSManager.hpp"

#include "Vector2D.hpp"

#include "./components/Motion_comp.hpp"
#include "./components/Render_comp.hpp"
#include "./components/Position_comp.hpp"



#include "raylib-cpp.hpp"

static std::map<std::string, raylib::Texture2D*> texture_repo;

void Render_init();

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
        
        raylib::Texture2D *texture_ptr = texture_repo[tex_loc];
        Vector2 tex_size = texture_ptr->GetSize();
          
        raylib::Rectangle src_rec(0.0, 0.0, tex_size.x, tex_size.y); // Use the entire texture size
        raylib::Rectangle dest_rec(world2screen_X(obj_pos.x), world2screen_Y(obj_pos.y), 
                                   world2screenscale_X(2*OBJECT_RADIUS), world2screenscale_Y(2*OBJECT_RADIUS));

        //origin is relative to dest_rec
        raylib::Vector2 origin = {world2screenscale_X(2*OBJECT_RADIUS)/2, world2screenscale_Y(2*OBJECT_RADIUS/2)};
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
        
        raylib::Texture2D *texture_ptr = texture_repo[tex_loc];
        Vector2 tex_size = texture_ptr->GetSize();
          
        raylib::Rectangle src_rec(0.0, 0.0, tex_size.x, tex_size.y); // Use the entire texture size
        raylib::Rectangle dest_rec(world2screen_X(obj_pos.x), world2screen_Y(obj_pos.y), 
                                   world2screenscale_X(2*OBJECT_RADIUS), world2screenscale_Y(2*OBJECT_RADIUS));
        
        //origin is relative to dest_rec
        raylib::Vector2 origin = {world2screenscale_X(2*OBJECT_RADIUS)/2, world2screenscale_Y(2*OBJECT_RADIUS/2)};
        texture_ptr->Draw(src_rec, dest_rec, origin, 0.0);

        raylib::Text ID_str(std::to_string(it->entity_id), 16.0, raylib::Color(255, 255, 255, 255));
        ID_str.Draw(world2screen_X(obj_pos.x), world2screen_Y(obj_pos.y));
           
    }
 
    DrawFPS(10,10);

}
