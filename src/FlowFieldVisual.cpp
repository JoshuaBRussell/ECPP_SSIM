#include "FlowFieldVisual.hpp"
#include "ECSManager.hpp"

#include <Eigen/Core>

#include "./components/Position_comp.hpp"
#include "./components/Render_comp.hpp"
#include "./components/Rotation_comp.hpp"
#include "./components/Vector_comp.hpp"

#include "../Examples/FlowField/main.hpp"

#include <cmath>

void FlowField_Visualization_System(ECS_Manager &world){
    
    
    double max_len = 0.0;
    for (auto it = world.get_component_begin<Vector_Component>(); 
              it < world.get_component_end<Vector_Component>(); it++){
        Eigen::Vector2d vec = world.get_component<Vector_Component>(it->entity_id)->vec; 
        double vec_mag = std::sqrt(vec(0)*vec(0) + vec(1)*vec(1));

        if (vec_mag > max_len){
            max_len = vec_mag;
        }
    }
    
    for (auto it = world.get_component_begin<Vector_Component>(); 
              it < world.get_component_end<Vector_Component>(); it++){
        // Take Physical coords. and convert to a location on the screen
        Eigen::Vector2d physical_pos = world.get_component<Position_Component>(it->entity_id)->position;
        
        Eigen::Vector2d vec = world.get_component<Vector_Component>(it->entity_id)->vec;
        double vec_mag = std::sqrt(vec(0)*vec(0) + vec(1)*vec(1));

        double scale = vec_mag/max_len;
        
        // TODO: This uses the main.hpp file to get screen_(width/height)_in_meters. These values should be passed into an init
        // function and saved as static variables by the system. Used the example's main.hpp as a temp measure
        world.get_component<Render_Component>(it->entity_id)->x = world2screen_X(physical_pos(0), SCREEN_WIDTH_METERS);
        world.get_component<Render_Component>(it->entity_id)->y = world2screen_Y(physical_pos(1), SCREEN_HEIGHT_METERS); 
        world.get_component<Render_Component>(it->entity_id)->width *= scale;
        world.get_component<Render_Component>(it->entity_id)->height *= scale; 
    }
}


