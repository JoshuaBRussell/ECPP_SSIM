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
#include "./ECS/components/PositionZ1_comp.hpp"
#include "./ECS/components/Position_comp.hpp"
#include "./ECS/components/Velocity_comp.hpp"
#include "./ECS/components/Acceleration_comp.hpp"
#include "./ECS/components/Force_comp.hpp"
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
    my_world.register_component<Vector_Component>(); 
    my_world.register_component<Particle_Component>(); 
    my_world.register_component<ODE_Component>();
    my_world.register_component<Force_Component>();
    my_world.register_component<Fixed_Rot_Component>(); 
    my_world.register_component<Relative_Rot_Component>(); // This isn't actually needed for this example
                                                           // but the program crashes if the component 
                                                           // isn't registered
    int entity_id = 1;
    // Display the Acceleration Field
    Eigen::Vector2f global_acc = Eigen::Vector2f(0.0, -0.81);  
    for(float x = -(SCREEN_WIDTH_METERS/2); x <= (SCREEN_WIDTH_METERS/2); x+= 1.0){
        for(float y = -(SCREEN_HEIGHT_METERS/2); y <= (SCREEN_HEIGHT_METERS/2); y+= 1.0){  
            Position_Component init_pos_val     = {entity_id, Eigen::Vector2f(x, y)};

            // Find the tangent rotation direction
            Eigen::Vector4f vec;
            Eigen::Vector4f tangent = ODE_Function(vec, global_acc);
            float angle = (180.0/3.14159) * std::atan2(tangent[3], tangent[1]);

            Vector_Component    init_vec_val    = {entity_id, Eigen::Vector2f(tangent[3], tangent[1])};
            Rotation_Component init_rot_val     = {entity_id, angle}; 
            Render_Component init_render_val    = {entity_id, "./misc/RedArrow.png",
                                                   320, 320, 50, 20}; // x, y, h, w

            my_world.add_component<Position_Component>(init_pos_val);
            my_world.add_component<Render_Component>(init_render_val);
            my_world.add_component<Rotation_Component>(init_rot_val);
            my_world.add_component<Vector_Component>(init_vec_val);
            entity_id++;
        } 
    }
    
    // For this example, this only needs to run once.
    FlowField_Visualization_System(my_world);
    
    
    int euler_id = entity_id;
    Particle_Component init_particle_flag = {euler_id};
    Position_Component init_particle_pos  = {euler_id, Eigen::Vector2f(-1.0, 0.0)};
    Velocity_Component init_particle_vel = {euler_id, Eigen::Vector2f(0.0, 0.2)}; 
    Rotation_Component init_rot_val       = {euler_id, 0.0}; 
    Render_Component init_render_val      = {euler_id, "./misc/RedCirc.png",
                                            320, 320, 20, 20}; // x, y, h, w; 
    ODE_Component init_ode_val            = {euler_id, INT_METHOD::EULER};
    Force_Component init_force_val        = {euler_id, Eigen::Vector2f(0.0, 0.0)};
    
    entity_id++; 
    Fixed_Rot_Component particle_constr = {entity_id, euler_id, Eigen::Vector2f(0.0, 0.0), 1.0};


    my_world.add_component<Particle_Component>(init_particle_flag);
    my_world.add_component<Position_Component>(init_particle_pos);
    my_world.add_component<Velocity_Component>(init_particle_vel); 
    my_world.add_component<Render_Component>(init_render_val);
    my_world.add_component<Rotation_Component>(init_rot_val);  
    my_world.add_component<ODE_Component>(init_ode_val); 
    my_world.add_component<Force_Component>(init_force_val); 
    my_world.add_component<Fixed_Rot_Component>(particle_constr);
    
    entity_id++;
    int rk_id = entity_id;
    Particle_Component init_particle_flag1 = {rk_id};
    Position_Component init_particle_pos1 = {rk_id, Eigen::Vector2f(-1.0, 0.0)};
    Velocity_Component init_particle_vel1 = {rk_id, Eigen::Vector2f(0.0, 0.2)}; 
    Rotation_Component init_rot_val1      = {rk_id, 0.0}; 
    Render_Component init_render_val1     = {rk_id, "./misc/BlueCirc.png",
                                            320, 320, 20, 20}; // x, y, h, w; 
    ODE_Component init_ode_val1           = {rk_id, INT_METHOD::RK4}; 
    Force_Component init_force_val1        = {rk_id, Eigen::Vector2f(0.0, 0.0)};
    entity_id++; 
    Fixed_Rot_Component particle1_constr = {entity_id, rk_id, Eigen::Vector2f(0.0, 0.0), 1.0}; 
    
    my_world.add_component<Particle_Component>(init_particle_flag1);
    my_world.add_component<Position_Component>(init_particle_pos1);
    my_world.add_component<Velocity_Component>(init_particle_vel1);
    my_world.add_component<Render_Component>(init_render_val1);
    my_world.add_component<Rotation_Component>(init_rot_val1); 
    my_world.add_component<ODE_Component>(init_ode_val1);
    my_world.add_component<Force_Component>(init_force_val1); 
    my_world.add_component<Fixed_Rot_Component>(particle1_constr); 
    

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

