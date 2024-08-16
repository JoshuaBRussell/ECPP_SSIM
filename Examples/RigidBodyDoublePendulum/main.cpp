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

#include "raylib-cpp.hpp"
#include "imgui.h"
#include "implot.h"
#include "rlImGui.h"

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

#define WORLD_RADIUS (SCREEN_WIDTH_METERS/2)

#define TOTAL_SUBSTEPS 8

#define TARGET_FPS 60.0

#define TEMP_DT (1/TARGET_FPS)

#define TWO_PI 6.2831853 // Only used for visualization modulo

#define WINDOW_NAME "Pendulum Visualization"

void add_rigid_body_to_world(ECS_Manager &world, int entity_id, Eigen::Vector2d pos, double angle){
    
    Particle_Component particle_flag      = {entity_id};
    Position_Component particle_pos       = {entity_id, pos};
    Velocity_Component particle_vel       = {entity_id, Eigen::Vector2d(0.0, 0.0)}; 
    Rotation_Component rot_val            = {entity_id, angle}; 
    Render_Component render_val           = {entity_id, "./misc/black_square.png",
                                                        SCREEN_WIDTH_IN_PIXELS/2, SCREEN_HEIGHT_IN_PIXELS/2, 
                                                        50, 200}; // x, y, h, w; 
    ODE_Component ode_val                 = {entity_id, INT_METHOD::RK4}; 
    Force_Component force_val             = {entity_id, Eigen::Vector2d(0.0, 0.0)};
    Mass_Component mass_val               = {entity_id, 1.0}; 
    Gravity_Component grav_val            = {entity_id}; 
    Torque_Component torque_val           = {entity_id, 0.0}; 
    Rot_Inertia_Component rot_inertia_val = {entity_id, 1.0};
    Angular_Vel_Component rot_vel_val     = {entity_id, 0.0};

    world.add_component<Particle_Component>(particle_flag);
    world.add_component<Position_Component>(particle_pos);
    world.add_component<Velocity_Component>(particle_vel);
    world.add_component<Render_Component>(render_val);
    world.add_component<Rotation_Component>(rot_val); 
    world.add_component<ODE_Component>(ode_val);
    world.add_component<Force_Component>(force_val); 
    world.add_component<Mass_Component>(mass_val); 
    world.add_component<Gravity_Component>(grav_val); 
    world.add_component<Torque_Component>(torque_val);
    world.add_component<Rot_Inertia_Component>(rot_inertia_val); 
    world.add_component<Angular_Vel_Component>(rot_vel_val);    
    
}

void add_fixed_pos_constr(ECS_Manager &world, 
                          int entity_id, int rb_id, 
                          Eigen::Vector2d world_pos, Eigen::Vector2d rel_pos){

    Fixed_Rot_Component fixed_rot_constr = {entity_id, rb_id, 
                                            world_pos, // world space point 
                                            rel_pos,  // body space  
                                            0.0}; 
    Render_Component init_constr_rend       = {entity_id, "./misc/blue_circle.png",
                                                          SCREEN_WIDTH_IN_PIXELS/2, SCREEN_HEIGHT_IN_PIXELS/2, 
                                                          15, 15}; 
    Position_Component init_constr_pos      = {entity_id, world_pos}; 
    Particle_Component init_particle_flag   = {entity_id}; 
    Rotation_Component init_constr_rot_val  = {entity_id, 1.5708};

    world.add_component<Fixed_Rot_Component>(fixed_rot_constr);
    world.add_component<Position_Component>(init_constr_pos); 
    world.add_component<Particle_Component>(init_particle_flag); 
    world.add_component<Render_Component>(init_constr_rend);
    world.add_component<Rotation_Component>(init_constr_rot_val);

}

void add_rel_constr(ECS_Manager &world, 
                     int entity_id, int rb1_id, int rb2_id, 
                     Eigen::Vector2d rel_pos1, Eigen::Vector2d rel_pos2, Eigen::Vector2d init_pos){
  
    Relative_Rot_Component rel_rot_constr = {entity_id, rb1_id, rb2_id, 
                                            rel_pos1, // body space - rigid body 1 
                                            rel_pos2, // body space - rigid body 2 
                                            0.0}; 
    Render_Component init_constr_rend2      = {entity_id, "./misc/blue_circle.png",
                                              SCREEN_WIDTH_IN_PIXELS/2, SCREEN_HEIGHT_IN_PIXELS/2, 
                                              15, 15}; 
    Position_Component init_constr_pos2     = {entity_id, init_pos}; 
    Particle_Component init_particle_flag4  = {entity_id}; 
    Rotation_Component init_constr_rot_val2     = {entity_id, 1.5708};

    world.add_component<Relative_Rot_Component>(rel_rot_constr);
    world.add_component<Position_Component>(init_constr_pos2); 
    world.add_component<Particle_Component>(init_particle_flag4); 
    world.add_component<Render_Component>(init_constr_rend2);
    world.add_component<Rotation_Component>(init_constr_rot_val2);

}


int main() {

    // Initialization
    raylib::Color textColor(LIGHTGRAY);
    raylib::Window w(SCREEN_WIDTH_IN_PIXELS, SCREEN_HEIGHT_IN_PIXELS, WINDOW_NAME);
    
    //SetTargetFPS(TARGET_FPS); 
     
    ECS_Manager my_world;

    // ---- Init Systems ---- //
    struct constr_visual_config constr_visual_config1 = {
        .screen_width_in_pixels  = SCREEN_WIDTH_IN_PIXELS,
        .screen_height_in_pixels = SCREEN_HEIGHT_IN_PIXELS,
        .screen_width_in_meters  = SCREEN_WIDTH_METERS,
        .screen_height_in_meters = SCREEN_HEIGHT_METERS
    };
    struct particle_visual_config particle_visual_config1 = {
        .screen_width_in_pixels  = SCREEN_WIDTH_IN_PIXELS,
        .screen_height_in_pixels = SCREEN_HEIGHT_IN_PIXELS,
        .screen_width_in_meters  = SCREEN_WIDTH_METERS,
        .screen_height_in_meters = SCREEN_HEIGHT_METERS
    };
     
    
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
                                              SCREEN_WIDTH_IN_PIXELS/2, SCREEN_HEIGHT_IN_PIXELS/2, 
                                              SCREEN_HEIGHT_IN_PIXELS , SCREEN_WIDTH_IN_PIXELS}; // x, y, h, w;
    Rotation_Component bg_rot_comp       = {bg_id, 0.0}; 
    my_world.add_component<Render_Component>(bg_render_comp);
    my_world.add_component<Rotation_Component>(bg_rot_comp); 
    
    int rb1_id = -1;
    int rb2_id = -1; 
    for (int i = 0; i < 2; i++){
        // First Rigid Body
        entity_id++;
        rb1_id = entity_id;
        add_rigid_body_to_world(my_world, rb1_id, Eigen::Vector2d(1.0, 0.0), 1.5707);
        
        // Second Rigid Body
        entity_id++;
        rb2_id = entity_id; 
        add_rigid_body_to_world(my_world, rb2_id, Eigen::Vector2d(2.0, -1.0), 0.0); 
        
        // Fixed Position Constraint
        entity_id++;
        int fixed_constr_id = entity_id;
        add_fixed_pos_constr(my_world, 
                             fixed_constr_id, rb1_id, 
                             Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 1.0)); 
        
        // Relative Position Constraint
        entity_id++;
        int rel_constr_id = entity_id;
        add_rel_constr(my_world, rel_constr_id, rb1_id, rb2_id,
                        Eigen::Vector2d(0.0, -1.0), Eigen::Vector2d(0.0, 1.0), Eigen::Vector2d(0.0, 1.0));
         
    } 
    // Initialize Systems after known established entites are created
    Constraint_Visualization_Init(constr_visual_config1);
    Particle_Visualization_Init(particle_visual_config1);
    Constraint_System_Init(my_world); 
    
    rlImGuiSetup(true);

    float x[3000];
    float y[3000];
    float y2[3000];
    for (int i = 0; i < 3000; i++){
        x[i] = i * 0.001f;
        y[i] = 0.5f + 0.5f * sinf(50*(x[i] + (float)ImGui::GetTime()/10));
    }

    int i = 0;
    float t = 0;
    while (!w.ShouldClose()) // Detect window close button or ESC key
    //while (i < 1)
    {
        i++; 
        //if (i%30 == 0){ 
        for (int i = 0; i < 100; i ++){
            Gravity_System(my_world); 
            Constraint_System(my_world);
            Newtonian_System(my_world, GetFrameTime()/100);
            
        }
        // Move the Constraint World Coord. so it can be seen
        Constraint_Visualization_System(my_world);

        // Converts Physical Coordinates to something the Render_System can use (Screen Coords)
        Particle_Visualization_System(my_world);

        Rotation_Component* rot_comp_ptr1 = my_world.get_component<Rotation_Component>(rb1_id);
        Rotation_Component* rot_comp_ptr2 = my_world.get_component<Rotation_Component>(rb2_id);

        BeginDrawing();
        ClearBackground(BLACK);
        Render_System(my_world);

        rlImGuiBegin();
        ImPlot::CreateContext();

        
        bool open = true;
        bool* p_open = &open;
        
        //ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_FirstUseEver);
        //ImGui::SetNextWindowSize(ImVec2(300, 50), ImGuiCond_FirstUseEver);
        
        ImGui::Begin("Data", p_open); 
        
        t += ImGui::GetIO().DeltaTime; 
        
        x[i%3000] = std::fmod(i * 0.001f, 3.0); 
        y[i%3000] = 0.5f + 0.5f * sinf(50*(x[i%3000] + (float)ImGui::GetTime()/10)); 
        y[i%3000]  = rot_comp_ptr1->angle;  
        float v = rot_comp_ptr2->angle;
        while (v >= M_PI) v -= TWO_PI;
        while (v < M_PI)  v += TWO_PI;  
        y2[i%3000] = v;

        if (ImPlot::BeginPlot("Line Plot")){
            ImPlot::SetupAxisLimits(ImAxis_X1,  0.0, 3.0); 
            ImPlot::SetupAxisLimits(ImAxis_Y1, -12, 12); 
            ImPlot::SetupAxes("x", "y");
            
            ImPlot::PlotLine("f(x)", x, y,  i%3000, 0, 0, sizeof(float));
            ImPlot::PlotLine("g(x)", x, y2, i%3000, 0, 0, sizeof(float)); 
            
            ImPlot::EndPlot();
        }
        
        ImPlot::DestroyContext();
        ImGui::End();
        rlImGuiEnd();

        EndDrawing();
        //} 
    }

    rlImGuiShutdown();
 
    return 0;
}

