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

#include "main.hpp"

#include "ECSManager.hpp"
#include "ComponentStorage.hpp"

#include "Motion.hpp"
#include "Rectangle.hpp"
#include "Render.hpp"
#include "Boundary.hpp"
#include "Controller.hpp"
#include "Collision.hpp"
#include "FlowFieldVisual.hpp"
#include "ParticleVisual.hpp"

#include "Vector2D.hpp"
#include "./ECS/components/Rotation_comp.hpp"
#include "./ECS/components/PositionZ1_comp.hpp"
#include "./ECS/components/Position_comp.hpp"
#include "./ECS/components/Velocity_comp.hpp"
#include "./ECS/components/Acceleration_comp.hpp"
#include "./ECS/components/Motion_comp.hpp"
#include "./ECS/components/Render_comp.hpp"
#include "./ECS/components/Boundary_comp.hpp"
#include "./ECS/components/Collision_comp.hpp"
#include "./ECS/components/Controller_comp.hpp"
#include "./ECS/components/Vector_comp.hpp"
#include "./ECS/components/Particle_comp.hpp"


#define WORLD_RADIUS (SCREEN_WIDTH_METERS/2)

#define TOTAL_SUBSTEPS 8

#define TARGET_FPS 60.0

#define TEMP_DT (1/TARGET_FPS)

#define WINDOW_NAME "Flow Field Visualization"

/*
static void add_new_ball(ECS_Manager &my_world, Vector2D pos, QuadTree &qt){
    
    int entity_id = my_world.create_entity();      
    
    PositionZ1_Component init_posz1_val = {entity_id, pos};
    Position_Component init_pos_val     = {entity_id, pos};
    //Velocity_Component init_vel_val     = {entity_id, Vector2D(entity_id*0.1, pow(entity_id, 1.02)*0.1)};
    float vx = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/3.0)) - 1.5; 
    float vy = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/3.0)) - 1.5;

    Velocity_Component init_vel_val     = {entity_id, Vector2D(vx, vy)};  
    Acceleration_Component init_acc_val = {entity_id, Vector2D(0.0, 0.0)}; 
    
    Controller_Component init_contr_val = {entity_id, 5.0, 0.0, Vector2D(2.0, 2.0)};
    
    Motion_Component init_mot_val       = {entity_id}; 
    Render_Component init_render_val    = {entity_id, "./misc/RedCirc.png"};
    Boundary_Component init_bounds_val  = {entity_id};
    Collision_Component init_coll_comp  = {entity_id, OBJECT_RADIUS};
    
    my_world.add_component<PositionZ1_Component>(init_posz1_val);
    my_world.add_component<Position_Component>(init_pos_val);
    my_world.add_component<Velocity_Component>(init_vel_val);
    my_world.add_component<Acceleration_Component>(init_acc_val); 

    my_world.add_component<Controller_Component>(init_contr_val);

    my_world.add_component<Motion_Component>(init_mot_val);
    my_world.add_component<Render_Component>(init_render_val); 
    my_world.add_component<Boundary_Component>(init_bounds_val);
    my_world.add_component<Collision_Component>(init_coll_comp);

    qt.add_element(entity_id, pos); 
}
*/


Vector2D ODE_Function(Vector2D x){
    
    Vector2D x_dot = Vector2D(0.0, 0.0);

    x_dot.x = x.y;
    x_dot.y = -0.5*x.x - x.y; 

    return x_dot;
}


int main() {

    // Initialization
    raylib::Color textColor(LIGHTGRAY);
    raylib::Window w(SCREEN_WIDTH_IN_PIXELS, SCREEN_HEIGHT_IN_PIXELS, WINDOW_NAME);
    raylib::Mouse Mouse;
    
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
    
    my_world.register_component<Render_Component>();
    my_world.register_component<Position_Component>();
    my_world.register_component<Collision_Component>();
    my_world.register_component<Rotation_Component>();
    my_world.register_component<Vector_Component>(); 
    my_world.register_component<Particle_Component>(); 
    
    int entity_id = 1; 
    
    for(float x = -(SCREEN_WIDTH_METERS/2); x <= (SCREEN_WIDTH_METERS/2); x+= 1.0){
        for(float y = -(SCREEN_HEIGHT_METERS/2); y <= (SCREEN_HEIGHT_METERS/2); y+= 1.0){  
            Position_Component init_pos_val     = {entity_id, Vector2D(x, y)};

            // Find the tangent rotation direction
            Vector2D tangent = ODE_Function(Vector2D(x, y));
            float angle = (180.0/3.14159) * std::atan2(tangent.y, tangent.x);

            Vector_Component    init_vec_val    = {entity_id, tangent};
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
    
    Particle_Component init_particle_flag = {entity_id};
    Position_Component init_particle_pos = {entity_id, Vector2D(-2.0, -2.0)};
    Rotation_Component init_rot_val      = {entity_id, 0.0}; 
    Render_Component init_render_val     = {entity_id, "./misc/BlueCirc.png",
                                            320, 320, 20, 20}; // x, y, h, w; 
    
    my_world.add_component<Particle_Component>(init_particle_flag);
    my_world.add_component<Position_Component>(init_particle_pos);
    my_world.add_component<Render_Component>(init_render_val);
    my_world.add_component<Rotation_Component>(init_rot_val);  
    
    while (!w.ShouldClose()) // Detect window close button or ESC key
    {
        Position_Component* pos_comp_ptr = my_world.get_component<Position_Component>(entity_id); 
        pos_comp_ptr->position += 0.0167*ODE_Function(pos_comp_ptr->position); 
        
        Particle_Visualization_System(my_world);

        BeginDrawing();
        ClearBackground(BLACK);
        Render_System(my_world);
        EndDrawing();
        
    }

    return 0;
}

