#include "Boids.hpp"

#include <algorithm>
#include <Eigen/Core>

#include "ECSManager.hpp"

#include "QuadTree.hpp"

#include "./components/Position_comp.hpp"
#include "./components/Velocity_comp.hpp"
#include "./components/Boundary_comp.hpp"
#include "./components/Collision_comp.hpp"


// Uses a QuadTree to improve spatial related queries. 
void Boids_QT(ECS_Manager &world, double dt, QuadTree &qt){

    for (auto coll_A = world.get_component_begin<Boundary_Component>(); 
              coll_A < world.get_component_end<Boundary_Component>(); coll_A++){

        Eigen::Vector2f seperation_vel_delta = Eigen::Vector2f(0.0, 0.0);
        Eigen::Vector2f neighbor_vel_sum     = Eigen::Vector2f(0.0, 0.0);
        Eigen::Vector2f neighbor_pos_sum     = Eigen::Vector2f(0.0, 0.0);

        int neighbor_count = 0;

        Velocity_Component *vel_comp_A_ptr = world.get_component<Velocity_Component>(coll_A->entity_id);  
        Position_Component *pos_comp_A_ptr = world.get_component<Position_Component>(coll_A->entity_id); 

        std::unordered_set<int>* neigh_set_ptr = qt.find_neighbors(coll_A->entity_id);
        std::unordered_set<int>::iterator it;
        for(it = neigh_set_ptr->begin(); it != neigh_set_ptr->end(); ++it){
            
            int coll_B_ID = *it; 
            
            if (coll_A->entity_id != coll_B_ID){
                Velocity_Component *vel_comp_B_ptr = world.get_component<Velocity_Component>(coll_B_ID);  
                Position_Component *pos_comp_B_ptr = world.get_component<Position_Component>(coll_B_ID);   
                
                // Find distance between to OBJECT_RADIUS
                Eigen::Vector2f disp_vec = pos_comp_A_ptr->position - pos_comp_B_ptr->position;
                double dist = disp_vec.norm();
                 
                if (dist <= VISUAL_RADIUS){
                    
                    neighbor_count+=1;
                    neighbor_vel_sum += vel_comp_B_ptr->velocity;
                    neighbor_pos_sum += pos_comp_B_ptr->position;
                      
                    if(dist <= PERSONAL_RADIUS){
                        seperation_vel_delta += (pos_comp_A_ptr->position - pos_comp_B_ptr->position); 
                    }
                    
                }
                
            }  

            // Update Velocities
            if(neighbor_count != 0){
                // Seperation  
                vel_comp_A_ptr->velocity += SEPERATION_GAIN*seperation_vel_delta;  
                
                //Alignment
                vel_comp_A_ptr->velocity += ALIGNMENT_GAIN*((1.0/neighbor_count)*neighbor_vel_sum);  
                
                // Cohesion
                Eigen::Vector2f neighbor_pos_avg = (1.0/neighbor_count)*neighbor_pos_sum;
                vel_comp_A_ptr->velocity += COHESION_GAIN*(neighbor_pos_avg - pos_comp_A_ptr->position);

                neighbor_count = 0;  
            }

            // Cap Velocity
            double vel_mag = vel_comp_A_ptr->velocity.norm(); 
            if (vel_mag > MAX_BOID_VEL){
                vel_comp_A_ptr->velocity = (MAX_BOID_VEL/vel_mag)*(vel_comp_A_ptr->velocity);
            }

        }

        delete neigh_set_ptr;

    }

    //qt.update();
}

// Uses a QuadTree to improve spatial related queries. 
void Boids_EdgeAvoidance(ECS_Manager &world, double dt){
    for (auto coll_A = world.get_component_begin<Boundary_Component>(); 
                  coll_A < world.get_component_end<Boundary_Component>(); coll_A++){
    
        Velocity_Component *vel_comp_A_ptr = world.get_component<Velocity_Component>(coll_A->entity_id);  
        Position_Component *pos_comp_A_ptr = world.get_component<Position_Component>(coll_A->entity_id); 

        double VEL_DELTA_MAG = 0.1;
        Eigen::Vector2f pos = pos_comp_A_ptr->position;
        double x_vel_delta = 0.0;
        double y_vel_delta = 0.0;
        if(pos(0) < 0.5){
            x_vel_delta = VEL_DELTA_MAG; 
        } else if (pos(0) > 3.5){
            x_vel_delta = -VEL_DELTA_MAG;
        }

        if(pos(1) < 0.5){
            y_vel_delta = VEL_DELTA_MAG; 
        } else if (pos(1) > 3.5){
            y_vel_delta = -VEL_DELTA_MAG;
        }

        vel_comp_A_ptr->velocity += Eigen::Vector2f(x_vel_delta, y_vel_delta);

    }
}
