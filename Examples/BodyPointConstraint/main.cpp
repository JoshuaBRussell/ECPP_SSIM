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
#include "Constraint.hpp"
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
    Render_Component bg_render_comp      =  {bg_id, "./misc/Background_Grid.png",
                                              320, 320, 640, 640}; // x, y, h, w;
    Rotation_Component bg_rot_comp       = {bg_id, 0.0}; 
    my_world.add_component<Render_Component>(bg_render_comp);
    my_world.add_component<Rotation_Component>(bg_rot_comp); 
    
    entity_id++;
    int rk_id = entity_id;

    Particle_Component init_particle_flag1 = {rk_id};
    Position_Component init_particle_pos1  = {rk_id, Eigen::Vector2f(2.0, 2.0)};
    Velocity_Component init_particle_vel1  = {rk_id, Eigen::Vector2f(0.0, 0.0)}; 
    Rotation_Component init_rot_val1       = {rk_id, 1.5708}; 
    Render_Component init_render_val1      = {rk_id, "./misc/BlueSquare.png",
                                              320, 320, 50, 200}; // x, y, h, w; 
    ODE_Component init_ode_val1            = {rk_id, INT_METHOD::RK4}; 
    Force_Component init_force_val1        = {rk_id, Eigen::Vector2f(0.0, 0.0)};
    Mass_Component init_mass_val1          = {rk_id, 1.0}; 
    Gravity_Component init_grav_val1       = {rk_id}; 
    Torque_Component init_torque_val1      = {rk_id, 0.0}; 
    Rot_Inertia_Component rot_inertia_val1 = {rk_id, 1.0};
    Angular_Vel_Component rot_vel_val1     = {rk_id, 0.0}; 
    
    entity_id++; 
    Fixed_Rot_Component particle1_constr = {entity_id, rk_id, 
                                            Eigen::Vector2f(1.0,  2.0), // world space point 
                                            Eigen::Vector2f(0.0, 1.0),  // body space  
                                            0.0}; 
    Render_Component init_render_val2      = {entity_id, "./misc/RedCirc.png",
                                              320, 320, 15, 15}; 
    Position_Component init_particle_pos2  = {entity_id, Eigen::Vector2f(1.0,  2.0)}; 
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
    
    // Convert the constrained body point position from body space to world space
    Eigen::Rotation2D<float> transform_matr = Eigen::Rotation2D<float>((3.14159/180.0)*-45.0);
    Eigen::Vector2f constr_body_pos = transform_matr * Eigen::Vector2f(-0.5, 0.0);
    std::cout << "X: " << constr_body_pos.x() << "Y: " << constr_body_pos.y() << std::endl;
    
     
    int i = 0;
    while (!w.ShouldClose()) // Detect window close button or ESC key
    //while (i < 1)
    {
        i++; 
        //if (i%30 == 0){ 
        for (int i = 0; i < 100; i ++){
            Gravity_System(my_world); 
            Constraint_System(my_world);
            Newtonian_System(my_world, TEMP_DT/100);
            
            Angular_Vel_Component* ang_vel_comp_ptr = my_world.get_component<Angular_Vel_Component>(rk_id);
            Rotation_Component* ang_comp_ptr = my_world.get_component<Rotation_Component>(rk_id); 
            //std::cout << "Angle: " << ang_comp_ptr->angle << " \n"; 
            //std::cout << "Angle Vel: " << ang_vel_comp_ptr ->w << " \n";

            // This normally gets reset in Newtonian Sys - not doing that so it
            // can be displayed for debug purposes.
            Force_Component* f_comp_ptr = my_world.get_component<Force_Component>(rk_id);
            Torque_Component* t_comp_ptr = my_world.get_component<Torque_Component>(rk_id); 
            
            f_comp_ptr->force = Eigen::Vector2f(0.0, 0.0);
            t_comp_ptr->torque = 0.0; 
        }
        
        // Converts Physical Coordinates to something the Render_System can use (Screen Coords)
        Particle_Visualization_System(my_world);

        BeginDrawing();
        ClearBackground(BLACK);
        Render_System(my_world);
        EndDrawing();
        //} 
    }
 
    return 0;
}

