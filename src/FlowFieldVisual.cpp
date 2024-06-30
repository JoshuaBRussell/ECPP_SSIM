#include "FlowFieldVisual.hpp"
#include "ECSManager.hpp"

#include <Eigen/Core>

#include "./components/Position_comp.hpp"
#include "./components/Render_comp.hpp"
#include "./components/Rotation_comp.hpp"
#include "./components/Vector_comp.hpp"

#include "../Examples/FlowField/main.hpp"

#include <cmath>

// ---- Util Functions ---- //

// The idea was that the scale functions would just transform the (...)scale_X/Y functions
// would handle the scale factor - esque conversions
//
// The (...)_X/Y functions would handle the transforms. 

// I don't like this and it either needs to change or be made more clear which is which.
static double screen2worldscale_X(int screen_x){
    return screen_x * ((double)SCREEN_WIDTH_METERS/SCREEN_WIDTH_IN_PIXELS); 
};

static double screen2worldscale_Y(int screen_y){
    return -(screen_y - SCREEN_HEIGHT_IN_PIXELS)*((double)SCREEN_HEIGHT_METERS/SCREEN_HEIGHT_IN_PIXELS);
};

static double screen2world_Y(int screen_y){
    return  -((double)SCREEN_HEIGHT_METERS/SCREEN_HEIGHT_IN_PIXELS) * (screen_y - SCREEN_HEIGHT_IN_PIXELS);
};

//Scale differences
static double world2screenscale_X(double x){
    return x * ((double)SCREEN_WIDTH_IN_PIXELS/SCREEN_WIDTH_METERS) + SCREEN_WIDTH_IN_PIXELS/2;  
}
static double world2screenscale_Y(double y){
    return y * ((double)SCREEN_HEIGHT_IN_PIXELS/SCREEN_HEIGHT_METERS) + SCREEN_HEIGHT_IN_PIXELS/2;
}

// Coord transform that assumes orthogonality for the transform
static double world2screen_X(double x){
    return world2screenscale_X(x);
}
static double world2screen_Y(double y){
    return -world2screenscale_Y(y) + SCREEN_HEIGHT_IN_PIXELS;
}

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
        
        world.get_component<Render_Component>(it->entity_id)->x = world2screen_X(physical_pos(0));
        world.get_component<Render_Component>(it->entity_id)->y = world2screen_Y(physical_pos(1));
        world.get_component<Render_Component>(it->entity_id)->width *= scale;
        world.get_component<Render_Component>(it->entity_id)->height *= scale; 
    }
}


