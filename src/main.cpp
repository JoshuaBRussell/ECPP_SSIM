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

#include "ECSManager.hpp"
#include "ComponentStorage.hpp"

#include "Motion.hpp"
#include "Rectangle.hpp"
#include "Render.hpp"
#include "Boundary.hpp"
#include "Controller.hpp"
#include "Collision.hpp"
#include "Boids.hpp"

#include "Vector2D.hpp"
#include "./ECS/components/PositionZ1_comp.hpp"
#include "./ECS/components/Position_comp.hpp"
#include "./ECS/components/Velocity_comp.hpp"
#include "./ECS/components/Acceleration_comp.hpp"
#include "./ECS/components/Motion_comp.hpp"
#include "./ECS/components/Render_comp.hpp"
#include "./ECS/components/Boundary_comp.hpp"
#include "./ECS/components/Collision_comp.hpp"
#include "./ECS/components/Controller_comp.hpp"

#include "QuadTree.hpp"

#define WORLD_RADIUS (SCREEN_WIDTH_METERS/2)

#define TOTAL_SUBSTEPS 8

#define TOTAL_OBJECTS 100 

#define TARGET_FPS 60.0

#define TEMP_DT (1/TARGET_FPS)

#define WINDOW_NAME "Virtual Bob"


static void add_new_ball(ECS_Manager &my_world, Vector2D pos, QuadTree &qt){
    
    int entity_id = my_world.create_entity();      
    
    PositionZ1_Component init_posz1_val = {entity_id, pos};
    Position_Component init_pos_val     = {entity_id, pos};
    //Velocity_Component init_vel_val     = {entity_id, Vector2D(entity_id*0.1, pow(entity_id, 1.02)*0.1)};
    float vx = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/3.0)) - 1.5; 
    float vy = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/3.0)) - 1.5;
    vx = 0.0;
    vy = 0.0;

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

int main() {

    // Initialization
    raylib::Color textColor(LIGHTGRAY);
    raylib::Window w(SCREEN_WIDTH_IN_PIXELS, SCREEN_HEIGHT_IN_PIXELS, WINDOW_NAME);
    raylib::Mouse Mouse;
    
    SetTargetFPS(TARGET_FPS); 
     
    ECS_Manager my_world;
    
    // Indicator Components - should these be 'archetypes'?
    my_world.register_component<Motion_Component>();
    my_world.register_component<Render_Component>();
    my_world.register_component<Boundary_Component>();
    my_world.register_component<Collision_Component>();
    
    my_world.register_component<Controller_Component>();

    my_world.register_component<PositionZ1_Component>();
    my_world.register_component<Position_Component>();
    my_world.register_component<Velocity_Component>();
    my_world.register_component<Acceleration_Component>();
    int i = 0;
    QuadTree my_qt(my_world, Vector2D(0.0, 0.0), Vector2D(4.0, 4.0), 2, 4); 
    raylib::Image *quad_image_ptr; 
    
    for (int i = 0; i < 0; i++){
        
        float x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/3.5)) + 0.25; 
        float y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/3.5)) + 0.25; 

        add_new_ball(my_world, Vector2D(x, y), my_qt); 
    }
 
    while (!w.ShouldClose()) // Detect window close button or ESC key
    {
        float mouse_x = screen2worldscale_X(Mouse.GetPosition().x);
        float mouse_y = screen2world_Y(Mouse.GetPosition().y);
        Vector2D mouse_pos = Vector2D(mouse_x, mouse_y); 
         
        if (Mouse.IsButtonPressed(0)){
            
            add_new_ball(my_world, mouse_pos, my_qt);
        }
        /* 
        i++;

        if (i%5 == 0){
            if (i < 15000){
                add_new_ball(my_world, mouse_pos, my_qt); 
            } 
        }
        */
        
        // Update
        for (int i = 0; i < 8; i++){
            //Controller_System(my_world); 
            Boids_QT(my_world, TEMP_DT/8, my_qt); 
            Motion_System(my_world, TEMP_DT/8);
            Collision_System_QT(my_world, TEMP_DT/8, my_qt); 
            //Collision_System(my_world, TEMP_DT/8);
            Boids_EdgeAvoidance(my_world, TEMP_DT/8);
            Boundary_System(my_world);
        }
        my_qt.update();

        BeginDrawing();
        ClearBackground(BLACK);
        // Draws Image files designated by the Render_Component
        //quad_image_ptr = my_qt.drawQuadTree(nullptr, Vector2D(0,0), 0);
        //raylib::Texture qt_tex(*quad_image_ptr); 
        //qt_tex.Draw();        
        Render_System_NonExclusive(my_world);
        EndDrawing();

        //qt_tex.Unload(); 
        
    }

    return 0;
}

