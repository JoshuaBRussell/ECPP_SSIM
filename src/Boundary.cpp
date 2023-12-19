#include "Boundary.hpp"

#include <Eigen/Core>

#include "ECSManager.hpp"

#include "./components/Position_comp.hpp"
#include "./components/Velocity_comp.hpp"
#include "./components/Collision_comp.hpp"
#include "./components/Boundary_comp.hpp"

void Boundary_System(ECS_Manager &world){
     
    for (auto it = world.get_component_begin<Boundary_Component>(); 
              it < world.get_component_end<Boundary_Component>(); it++){

        Position_Component *pos_comp_ptr = world.get_component<Position_Component>(it->entity_id);
        Collision_Component *coll_comp_ptr = world.get_component<Collision_Component>(it->entity_id);

        if((pos_comp_ptr->position(0)  - coll_comp_ptr->radius)< 0){
            pos_comp_ptr->position(0) = coll_comp_ptr->radius;
            world.get_component<Velocity_Component>(it->entity_id)->velocity(0) *= -0.75;
        }
        if(pos_comp_ptr->position(0) + coll_comp_ptr->radius> 4.0){
            pos_comp_ptr->position(0) = 4.0 - coll_comp_ptr->radius;
            world.get_component<Velocity_Component>(it->entity_id)->velocity(0) *= -0.75;
        }
        if((pos_comp_ptr->position(1) - coll_comp_ptr->radius) < 0){
            pos_comp_ptr->position(1) = coll_comp_ptr->radius;
            world.get_component<Velocity_Component>(it->entity_id)->velocity(1) *= -0.75;
        }
        if(pos_comp_ptr->position(1) + coll_comp_ptr->radius> 4.0){
            pos_comp_ptr->position(1) = 4.0 - coll_comp_ptr->radius;
            world.get_component<Velocity_Component>(it->entity_id)->velocity(1) *= -0.75;
        }
    }
}
