#include <iostream>
#include <vector>
#include <map>
#include <typeinfo>
#include <memory>
#include <array>
#include <set>
#include <string>

#include <math.h>

#include <raylib-cpp.hpp>

#include <Eigen/Dense>

#include "main.hpp"

#include "ECSManager.hpp"
#include "ComponentStorage.hpp"

#include "Newtonian_Sys.hpp"
#include "Motion.hpp"
#include "Rectangle.hpp"
#include "Render.hpp"
#include "Boundary.hpp"
#include "Controller.hpp"
#include "Collision.hpp"
#include "FlowFieldVisual.hpp"
#include "ParticleVisual.hpp"
#include "Constraint.hpp"

#include "./ECS/components/Rotation_comp.hpp"
#include "./ECS/components/Angular_Vel_comp.hpp"
#include "./ECS/components/Rot_Inertia_comp.hpp"
#include "./ECS/components/Inertia_comp.hpp"
#include "./ECS/components/PositionZ1_comp.hpp"
#include "./ECS/components/Position_comp.hpp"
#include "./ECS/components/Velocity_comp.hpp"
#include "./ECS/components/Acceleration_comp.hpp"
#include "./ECS/components/Force_comp.hpp"
#include "./ECS/components/Torque_comp.hpp"
#include "./ECS/components/Motion_comp.hpp"
#include "./ECS/components/Render_comp.hpp"
#include "./ECS/components/Boundary_comp.hpp"
#include "./ECS/components/Collision_comp.hpp"
#include "./ECS/components/Controller_comp.hpp"
#include "./ECS/components/Vector_comp.hpp"
#include "./ECS/components/Particle_comp.hpp"
#include "./ECS/components/ODE_comp.hpp"
#include "./ECS/components/Constraint_comp.hpp"

#define WORLD_RADIUS (SCREEN_WIDTH_METERS/2)

#define TOTAL_SUBSTEPS 8

#define TARGET_FPS 60.0

#define TEMP_DT (1/TARGET_FPS)

#define WINDOW_NAME "Pendulum Visualization"


int main() {

    // Initialization
    raylib::Color textColor(LIGHTGRAY);
    raylib::Window w(SCREEN_WIDTH_IN_PIXELS, SCREEN_HEIGHT_IN_PIXELS, WINDOW_NAME);
    
    SetTargetFPS(TARGET_FPS); 
     
    ECS_Manager my_world;

    // ---- Init Systems ---- //
    struct render_config render_config = {
        .screen_width_in_pixels  = SCREEN_WIDTH_IN_PIXELS,
        .screen_height_in_pixels = SCREEN_HEIGHT_IN_PIXELS,
        .screen_width_in_meters  = SCREEN_WIDTH_METERS,
        .screen_height_in_meters = SCREEN_HEIGHT_METERS
    };
    
    Render_init(render_config);
    Constraint_System_Init(my_world);

    my_world.register_component<Render_Component>();
    my_world.register_component<Position_Component>();
    my_world.register_component<Velocity_Component>(); 
    my_world.register_component<Motion_Component>(); 
    my_world.register_component<Collision_Component>();
    my_world.register_component<Rotation_Component>();
    my_world.register_component<Angular_Vel_Component>();
    my_world.register_component<Inertia_Component>(); 
    my_world.register_component<Rot_Inertia_Component>();
    my_world.register_component<Vector_Component>(); 
    my_world.register_component<Particle_Component>(); 
    my_world.register_component<ODE_Component>();
    my_world.register_component<Force_Component>();
    my_world.register_component<Torque_Component>(); 
    my_world.register_component<Fixed_Rot_Component>(); 
    my_world.register_component<Relative_Rot_Component>(); // This isn't actually needed for this example
                                                           // but the program crashes if the component 
                                                           // isn't registered
    int entity_id = 1;
    
    Particle_Component init_particle_flag  = {entity_id};
    Position_Component init_particle_pos   = {entity_id, Eigen::Vector2f(0.0, 0.0)};
    Velocity_Component init_particle_vel   = {entity_id, Eigen::Vector2f(0.0, 0.0)}; 
    Rotation_Component init_rot_val        = {entity_id, 0.0}; 
    Angular_Vel_Component init_rot_vel_val = {entity_id, 0.0}; 
    Render_Component init_render_val       = {entity_id, "./misc/BlueSquare.png",
                                            320, 320, 100, 100}; // x, y, h, w; 
    ODE_Component init_ode_val             = {entity_id, INT_METHOD::EULER};
    Force_Component init_force_val         = {entity_id, Eigen::Vector2f(0.0, 0.0)};
    Torque_Component init_torque_val        = {entity_id, 11.0}; 
    Inertia_Component init_inertia_val     = {entity_id, 1.0};
    Rot_Inertia_Component init_rot_inertia_val = {entity_id, 1.0}; 
    
    my_world.add_component<Particle_Component>(init_particle_flag);
    my_world.add_component<Position_Component>(init_particle_pos);
    my_world.add_component<Velocity_Component>(init_particle_vel); 
    my_world.add_component<Render_Component>(init_render_val);
    my_world.add_component<Rotation_Component>(init_rot_val);  
    my_world.add_component<ODE_Component>(init_ode_val); 
    my_world.add_component<Force_Component>(init_force_val); 
    my_world.add_component<Torque_Component>(init_torque_val); 
    my_world.add_component<Angular_Vel_Component>(init_rot_vel_val);
    
    my_world.add_component<Inertia_Component>(init_inertia_val); 
    my_world.add_component<Rot_Inertia_Component>(init_rot_inertia_val); 

    while (!w.ShouldClose()) // Detect window close button or ESC key
    {
        
        for (int i = 0; i < 25; i ++){
            Constraint_System(my_world); 
            Newtonian_System(my_world, TEMP_DT/25);
        }

        Particle_Visualization_System(my_world);

        BeginDrawing();
        ClearBackground(BLACK);
        Render_System(my_world);
        EndDrawing();
        
    }

    return 0;
}

