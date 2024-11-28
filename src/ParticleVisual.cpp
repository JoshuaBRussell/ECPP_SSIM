#include "ParticleVisual.hpp"
#include "ECSManager.hpp"

#include <Eigen/Core>

#include "ECS/Render.hpp"

#include "./components/Particle_comp.hpp"
#include "./components/Position_comp.hpp"
#include "./components/Render_comp.hpp"


#include <cmath>


static int screen_width_in_pixels = 0;
static int screen_height_in_pixels = 0;

static double screen_width_in_meters = 0.0;
static double screen_height_in_meters = 0.0;


// ---- ---- //

void Particle_Visualization_Init(struct particle_visual_config &particle_visual_config){

    screen_width_in_pixels  = particle_visual_config.screen_width_in_pixels;
    screen_height_in_pixels = particle_visual_config.screen_height_in_pixels;
    screen_width_in_meters  = particle_visual_config.screen_width_in_meters;
    screen_height_in_meters = particle_visual_config.screen_height_in_meters;
    
}

void Particle_Visualization_System(ECS_Manager &world){
    
    for (auto it = world.get_component_begin<Particle_Component>(); 
              it < world.get_component_end<Particle_Component>(); it++){
        // Take Physical coords. and convert to a location on the screen
        Eigen::Vector2d physical_pos = world.get_component<Position_Component>(it->entity_id)->position;
        
        world.get_component<Render_Component>(it->entity_id)->x = world2screen_X(physical_pos(0), screen_width_in_meters);
        world.get_component<Render_Component>(it->entity_id)->y = world2screen_Y(physical_pos(1), screen_height_in_meters);
    }
}


