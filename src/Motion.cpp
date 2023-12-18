#include "Motion.hpp"

#include "ECSManager.hpp"

#include <Eigen/Core>

#include "./components/Motion_comp.hpp"
#include "./components/PositionZ1_comp.hpp"
#include "./components/Position_comp.hpp"
#include "./components/Velocity_comp.hpp"
#include "./components/Acceleration_comp.hpp"



void Motion_System(ECS_Manager &world, float dt){

    for (auto it = world.get_component_begin<Motion_Component>(); 
              it < world.get_component_end<Motion_Component>(); it++){ 
        
        Eigen::Vector2f temp_pos = world.get_component<Position_Component>(it->entity_id)->position;
        
        /*
        Eigen::Vector2f new_pos  = world.get_component<Position_Component>(it->entity_id)->position + 
                   dt*(world.get_component<Velocity_Component>(it->entity_id)->velocity) + 
                   dt*dt * (world.get_component<Acceleration_Component>(it->entity_id)->accel);  
        */
        /*
        Eigen::Vector2f new_pos = 2*world.get_component<Position_Component>(it->entity_id)->position - 
                             world.get_component<PositionZ1_Component>(it->entity_id)->position +
                             dt*dt * (world.get_component<Acceleration_Component>(it->entity_id)->accel);   
        */

        Eigen::Vector2f new_pos = world.get_component<Position_Component>(it->entity_id)->position +
                           dt*world.get_component<Velocity_Component>(it->entity_id)->velocity +
                           0.5*dt*dt*world.get_component<Acceleration_Component>(it->entity_id)->accel;
        world.get_component<Position_Component>(it->entity_id)->position   = new_pos; 
        world.get_component<PositionZ1_Component>(it->entity_id)->position = temp_pos;

        Eigen::Vector2f new_vel = world.get_component<Velocity_Component>(it->entity_id)->velocity + 
                  dt*world.get_component<Acceleration_Component>(it->entity_id)->accel;
       
        world.get_component<Velocity_Component>(it->entity_id)->velocity = new_vel;
        //world.get_component<Velocity_Component>(it->entity_id)->velocity = (1/dt)*(world.get_component<Position_Component>(it->entity_id)->position - 
        //                                                world.get_component<PositionZ1_Component>(it->entity_id)->position);
    }

}
