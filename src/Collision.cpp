#include "Collision.hpp"


#include "ECSManager.hpp"

#include "./components/Position_comp.hpp"
#include "./components/Velocity_comp.hpp"
#include "./components/Boundary_comp.hpp"
#include "./components/Collision_comp.hpp"

void Collision_System(ECS_Manager &world, double dt){

    for (auto coll_A = world.get_component_begin<Boundary_Component>(); 
              coll_A < world.get_component_end<Boundary_Component>(); coll_A++){

        Position_Component *pos_comp_A_ptr = world.get_component<Position_Component>(coll_A->entity_id);  
        for (auto coll_B = world.get_component_begin<Boundary_Component>(); 
              coll_B < world.get_component_end<Boundary_Component>(); coll_B++){
             
            if (coll_A != coll_B){
                Position_Component *pos_comp_B_ptr = world.get_component<Position_Component>(coll_B->entity_id);   
                // Find distance between to OBJECT_RADIUS
                Vector2D disp_vec = pos_comp_A_ptr->position - pos_comp_B_ptr->position;
                double dist = disp_vec.mag();
                // If the distance is less the sum of their radii -> collision occured
                
                if ( dist <= 2*OBJECT_RADIUS){
                    Vector2D norm_vec = (1/dist)*(disp_vec);

                    Vector2D temp = (2*OBJECT_RADIUS - dist) * norm_vec;
                    
                    pos_comp_A_ptr->position += temp;
                    pos_comp_B_ptr->position -= temp; 
                    
                    // Update Velocities
                     
                    Velocity_Component *vel_comp_A_ptr = world.get_component<Velocity_Component>(coll_A->entity_id);  
                    Velocity_Component *vel_comp_B_ptr = world.get_component<Velocity_Component>(coll_B->entity_id);
                    
                    Vector2D temp_vel = vel_comp_A_ptr->velocity;
                    vel_comp_A_ptr->velocity = vel_comp_B_ptr->velocity;
                    vel_comp_B_ptr->velocity = temp_vel; 
                    

                }
                
            }
        }
    }
}
