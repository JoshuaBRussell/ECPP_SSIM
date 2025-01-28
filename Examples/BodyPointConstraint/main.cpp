#include <iostream>
#include <vector>
#include <map>
#include <typeinfo>
#include <memory>
#include <array>
#include <set>
#include <string>

#include <math.h>
#include <cmath>
#include <raylib-cpp.hpp>

#include <Eigen/Dense>

#include "Mass_comp.hpp"
#include "main.hpp"

#include "ECSManager.hpp"
#include "ComponentStorage.hpp"

#include "Newtonian_Sys.hpp"
#include "Gravity_Sys.hpp"
#include "Motion.hpp"
#include "Rectangle.hpp"
#include "Render.hpp"
#include "Boundary.hpp"
#include "Controller.hpp"
#include "Collision.hpp"
#include "FlowFieldVisual.hpp"
#include "ParticleVisual.hpp"
#include "ConstraintVisual.hpp"
#include "Constraint.hpp"

#include "./ECS/components/Rotation_comp.hpp"
#include "./ECS/components/PositionZ1_comp.hpp"
#include "./ECS/components/Position_comp.hpp"
#include "./ECS/components/Velocity_comp.hpp"
#include "./ECS/components/Acceleration_comp.hpp"
#include "./ECS/components/Force_comp.hpp"
#include "./ECS/components/Torque_comp.hpp"
#include "./ECS/components/Rot_Inertia_comp.hpp"
#include "./ECS/components/Angular_Vel_comp.hpp"
#include "./ECS/components/Motion_comp.hpp"
#include "./ECS/components/Render_comp.hpp"
#include "./ECS/components/Boundary_comp.hpp"
#include "./ECS/components/Collision_comp.hpp"
#include "./ECS/components/Controller_comp.hpp"
#include "./ECS/components/Vector_comp.hpp"
#include "./ECS/components/Particle_comp.hpp"
#include "./ECS/components/ODE_comp.hpp"
#include "./ECS/components/Constraint_comp.hpp"
#include "./ECS/components/Gravity_comp.hpp"
#include "./ECS/components/Connector_comp.hpp"
#include "RigidBodyUtil.hpp"
#include "raylib.h"

#define WORLD_RADIUS (SCREEN_WIDTH_METERS/2)

#define TOTAL_SUBSTEPS 8

#define TARGET_FPS 60

#define TEMP_DT (1/TARGET_FPS)

#define WINDOW_NAME "Pendulum Visualization"


int main() {

    // Initialization
    ECS_Manager my_world;

    my_world.register_component<Render_Component>();
    my_world.register_component<Position_Component>();
    my_world.register_component<Velocity_Component>(); 
    my_world.register_component<Motion_Component>(); 
    my_world.register_component<Collision_Component>();
    my_world.register_component<Rotation_Component>();
    my_world.register_component<Vector_Component>(); 
    my_world.register_component<Particle_Component>(); 
    my_world.register_component<ODE_Component>();
    my_world.register_component<Force_Component>();
    my_world.register_component<Mass_Component>();
    my_world.register_component<Torque_Component>(); 
    my_world.register_component<Rot_Inertia_Component>();
    my_world.register_component<Angular_Vel_Component>(); 
    my_world.register_component<Gravity_Component>();
    my_world.register_component<Connector_Component>();
    my_world.register_component<Fixed_Rot_Component>(); 
    my_world.register_component<Relative_Rot_Component>(); // This isn't actually needed for this example
                                                           // but the program crashes if the component 
                                                           // isn't registered
    int entity_id = 1;
    // Create the Background
    int bg_id = entity_id;
    Render_Component bg_render_comp      =  {bg_id, "./misc/background_w_grid.png",
                                              320, 320, 640, 640}; // x, y, h, w;
    Rotation_Component bg_rot_comp       = {bg_id, 0.0}; 
    my_world.add_component<Render_Component>(bg_render_comp);
    my_world.add_component<Rotation_Component>(bg_rot_comp); 
    
    entity_id++;
    int rk_id = entity_id;

    Particle_Component init_particle_flag1 = {rk_id};
    Position_Component init_particle_pos1  = {rk_id, Eigen::Vector2d(2.0, 2.0)};
    Velocity_Component init_particle_vel1  = {rk_id, Eigen::Vector2d(0.0, 0.0)}; 
    Rotation_Component init_rot_val1       = {rk_id, 1.5708}; 
    Render_Component init_render_val1      = {rk_id, "./misc/blue_circle.png",
                                              320, 320, 50, 200}; // x, y, h, w; 
    ODE_Component init_ode_val1            = {rk_id, INT_METHOD::RK4}; 
    Force_Component init_force_val1        = {rk_id, Eigen::Vector2d(0.0, 0.0)};
    Mass_Component init_mass_val1          = {rk_id, 1.0}; 
    Gravity_Component init_grav_val1       = {rk_id}; 
    Torque_Component init_torque_val1      = {rk_id, 0.0}; 
    Rot_Inertia_Component rot_inertia_val1 = {rk_id, 1.0};
    Angular_Vel_Component rot_vel_val1     = {rk_id, 0.0}; 

    entity_id++; 
    Fixed_Rot_Component particle1_constr = {entity_id, rk_id, 
                                            Eigen::Vector2d(1.0,  2.0), // world space point 
                                            Eigen::Vector2d(0.0, 1.0),  // body space  
                                            0.0}; 
    Render_Component init_render_val2      = {entity_id, "./misc/red_circle.png",
                                              320, 320, 15, 15}; 
    Position_Component init_particle_pos2  = {entity_id, Eigen::Vector2d(1.0,  2.0)}; 
    Particle_Component init_particle_flag2 = {entity_id}; 
    Rotation_Component init_rot_val2       = {entity_id, 1.5708};

    my_world.add_component<Particle_Component>(init_particle_flag1);
    my_world.add_component<Position_Component>(init_particle_pos1);
    my_world.add_component<Velocity_Component>(init_particle_vel1);
    my_world.add_component<Render_Component>(init_render_val1);
    my_world.add_component<Rotation_Component>(init_rot_val1); 
    my_world.add_component<ODE_Component>(init_ode_val1);
    my_world.add_component<Force_Component>(init_force_val1); 
    my_world.add_component<Mass_Component>(init_mass_val1); 
    my_world.add_component<Gravity_Component>(init_grav_val1); 
    my_world.add_component<Torque_Component>(init_torque_val1);
    my_world.add_component<Rot_Inertia_Component>(rot_inertia_val1); 
    my_world.add_component<Angular_Vel_Component>(rot_vel_val1);    
 
    my_world.add_component<Fixed_Rot_Component>(particle1_constr);
    my_world.add_component<Position_Component>(init_particle_pos2); 
    my_world.add_component<Particle_Component>(init_particle_flag2); 
    my_world.add_component<Render_Component>(init_render_val2);
    my_world.add_component<Rotation_Component>(init_rot_val2);
    
    


    // Initialize Systems after known established entites are created  
     
    // ---- Init Systems ---- //
    struct constr_visual_config constr_visual_config = {
        .screen_width_in_pixels  = SCREEN_WIDTH_IN_PIXELS,
        .screen_height_in_pixels = SCREEN_HEIGHT_IN_PIXELS,
        .screen_width_in_meters  = SCREEN_WIDTH_METERS,
        .screen_height_in_meters = SCREEN_HEIGHT_METERS
    };
    struct particle_visual_config particle_visual_config = {
        .screen_width_in_pixels  = SCREEN_WIDTH_IN_PIXELS,
        .screen_height_in_pixels = SCREEN_HEIGHT_IN_PIXELS,
        .screen_width_in_meters  = SCREEN_WIDTH_METERS,
        .screen_height_in_meters = SCREEN_HEIGHT_METERS
    }; 
    
    struct render_config render_config = {
        .screen_width_in_pixels  = SCREEN_HEIGHT_IN_PIXELS,
        .screen_height_in_pixels = SCREEN_HEIGHT_IN_PIXELS,
        .window_title            = WINDOW_NAME, 
        .target_fps              = TARGET_FPS
    };

    
    Constraint_System_Init(my_world); 
    Render_System_Init(my_world, render_config); 

    Constraint_Visualization_Init(constr_visual_config);
    Particle_Visualization_Init(particle_visual_config); 
    
    
    // Move the Constraint World Coord. so it can be seen 
    Render_System_add_pre_render(Constraint_Visualization_System); 
    // Converts Physical Coordinates to something the Render_System can use (Screen Coords) 
    Render_System_add_pre_render(Particle_Visualization_System);


    while (!Render_System_WindowShouldClose()) // Detect window close button or ESC key
    {
        for (int i = 0; i < 100; i ++){
            Gravity_System(my_world); 
            Constraint_System(my_world);
            Newtonian_System(my_world, GetFrameTime()/100);
            
            Angular_Vel_Component* ang_vel_comp_ptr = my_world.get_component<Angular_Vel_Component>(rk_id);
            Rotation_Component* ang_comp_ptr = my_world.get_component<Rotation_Component>(rk_id); 
            //std::cout << "Angle: " << ang_comp_ptr->angle << " \n"; 
            //std::cout << "Angle Vel: " << ang_vel_comp_ptr ->w << " \n";

            // This normally gets reset in Newtonian Sys - not doing that so it
            // can be displayed for debug purposes.
            Force_Component* f_comp_ptr = my_world.get_component<Force_Component>(rk_id);
            Torque_Component* t_comp_ptr = my_world.get_component<Torque_Component>(rk_id); 
            
            f_comp_ptr->force = Eigen::Vector2d(0.0, 0.0);
            t_comp_ptr->torque = 0.0; 
        }
        
        Render_System(my_world);
    }
 
    return 0;
}

