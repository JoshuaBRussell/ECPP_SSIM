#include "Collision.hpp"

#include <Eigen/Core>

#include "ECSManager.hpp"

#include "QuadTree.hpp"

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
                Eigen::Vector2f disp_vec = pos_comp_A_ptr->position - pos_comp_B_ptr->position;
                double dist = disp_vec.norm();
                
                // If the distance is less the sum of their radii -> collision occured
                double obj_radius = world.get_component<Collision_Component>(coll_B->entity_id)->radius;  
                
                if ( dist <= 2*obj_radius){
                    Eigen::Vector2f norm_vec = (1/dist)*(disp_vec);

                    Eigen::Vector2f temp = (2*obj_radius - dist) * norm_vec;
                    
                    pos_comp_A_ptr->position += temp;
                    pos_comp_B_ptr->position -= temp; 
                    
                    // Update Velocities
                     
                    Velocity_Component *vel_comp_A_ptr = world.get_component<Velocity_Component>(coll_A->entity_id);  
                    Velocity_Component *vel_comp_B_ptr = world.get_component<Velocity_Component>(coll_B->entity_id);
                    
                    Eigen::Vector2f temp_vel = vel_comp_A_ptr->velocity;
                    vel_comp_A_ptr->velocity = vel_comp_B_ptr->velocity;
                    vel_comp_B_ptr->velocity = temp_vel; 
                    

                }
                
            }
        }
    }
}

// Uses a QuadTree to improve spatial related queries. 
// Rather than iterating through the list of Collision Components,
// it iterates through the leaves of the QT to (hopefully) improve cache performance.
void Collision_System_QT(ECS_Manager &world, double dt, QuadTree &qt){

    for (auto coll_A = world.get_component_begin<Boundary_Component>(); 
              coll_A < world.get_component_end<Boundary_Component>(); coll_A++){ 
        
        Position_Component *pos_comp_A_ptr = world.get_component<Position_Component>(coll_A->entity_id); 
        
        std::unordered_set<int>* neigh_set_ptr = qt.find_neighbors(coll_A->entity_id);
        std::unordered_set<int>::iterator it;
        for(it = neigh_set_ptr->begin(); it != neigh_set_ptr->end(); ++it){
            
            int coll_B_ID = *it; 
            
            if (coll_A->entity_id != coll_B_ID){
                Position_Component *pos_comp_B_ptr = world.get_component<Position_Component>(coll_B_ID);   
                // Find distance between to OBJECT_RADIUS
                Eigen::Vector2f disp_vec = pos_comp_A_ptr->position - pos_comp_B_ptr->position;
                double dist = disp_vec.norm();
                // If the distance is less the sum of their radii -> collision occured

                double obj_radius = world.get_component<Collision_Component>(coll_B_ID)->radius;
                
                if ( dist <= 2*obj_radius){
                    Eigen::Vector2f norm_vec = (1/dist)*(disp_vec);

                    Eigen::Vector2f temp = (2*obj_radius - dist) * norm_vec;
                    
                    pos_comp_A_ptr->position += temp;
                    pos_comp_B_ptr->position -= temp; 
                    
                    // Update Velocities
                     
                    Velocity_Component *vel_comp_A_ptr = world.get_component<Velocity_Component>(coll_A->entity_id);  
                    Velocity_Component *vel_comp_B_ptr = world.get_component<Velocity_Component>(coll_B_ID);
                    
                    Eigen::Vector2f temp_vel = vel_comp_A_ptr->velocity;
                    vel_comp_A_ptr->velocity = vel_comp_B_ptr->velocity;
                    vel_comp_B_ptr->velocity = temp_vel; 
                    

                }
                
            }  
        }

        delete neigh_set_ptr;

    }

    //qt.update();
}
