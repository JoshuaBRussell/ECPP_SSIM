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
#include "Render.hpp"
#include "Boundary.hpp"
#include "Controller.hpp"
#include "Collision.hpp"

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

#define TOTAL_OBJECTS 50 

#define TARGET_FPS 60.0

#define TEMP_DT (1/TARGET_FPS)

#define WINDOW_NAME "Virtual Bob"


static void add_new_ball(ECS_Manager &my_world, Vector2D pos, QuadTree qt){
    
    int entity_id = my_world.create_entity();
    qt.add_element(entity_id, pos);
    
    PositionZ1_Component init_posz1_val = {entity_id, pos};
    Position_Component init_pos_val     = {entity_id, pos};
    Velocity_Component init_vel_val     = {entity_id, Vector2D(entity_id*0.1, pow(entity_id, 1.2)*0.1)};
    Acceleration_Component init_acc_val = {entity_id, Vector2D(0.0, -0.81)}; 
    
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

    QuadTree my_qt(my_world, Vector2D(0.0, 0.0), Vector2D(4.0, 4.0), 16, 4);  

    // Main game loop
    while (!w.ShouldClose()) // Detect window close button or ESC key
    {
        
        if (Mouse.IsButtonPressed(0)){
            float x = screen2worldscale_X(Mouse.GetPosition().x);
            float y = screen2world_Y(Mouse.GetPosition().y);
            Vector2D pos = Vector2D(x, y);
            add_new_ball(my_world, pos, my_qt);
        }
        // Update
        for (int i = 0; i < 8; i++){
            Controller_System(my_world); 
            //Motion_System(my_world, TEMP_DT/8); 
            Collision_System(my_world, TEMP_DT/8);  
            Boundary_System(my_world); 
        }

        Render_System(my_world);
        
    }

    return 0;
}

