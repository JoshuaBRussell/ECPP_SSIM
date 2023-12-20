#include "Motion.hpp"

#include "ECSManager.hpp"

#include <Eigen/Core>

#include "./components/Constraint_comp.hpp"
#include "./components/Position_comp.hpp"
#include "./components/Velocity_comp.hpp"
#include "./components/Acceleration_comp.hpp"
#include "./components/Force_comp.hpp"

void Constraint_System(ECS_Manager &world){

    for (auto it = world.get_component_begin<Constraint_Component>(); 
              it < world.get_component_end<Constraint_Component>(); it++){ 
        
        Position_Component* pos_comp_ptr = world.get_component<Position_Component>(it->entity_id);
        Velocity_Component* vel_comp_ptr = world.get_component<Velocity_Component>(it->entity_id); 
        Force_Component* force_comp_ptr = world.get_component<Force_Component>(it->entity_id);
        
        // Find Constraint Force
        // Assume that gravity is a thing
        Eigen::Vector2f Force_App = Eigen::Vector2f(0.0, -0.81); // g = 9.81 m/s^2 | m = 1
        
        Eigen::Vector2f pos_vec = pos_comp_ptr->position;
        Eigen::Vector2f vel_vec = vel_comp_ptr->velocity;

        double lambda_not = (-(vel_vec(0)*vel_vec(0) + vel_vec(1)*vel_vec(1)) - (pos_vec(0)*Force_App(0) + pos_vec(1)*Force_App(1)));
        double lambda = lambda_not/(pos_vec(0)*pos_vec(0) +pos_vec(1)*pos_vec(1));

        Eigen::Vector2f Force_Constraint = lambda * pos_vec;

        force_comp_ptr->force = Force_App + Force_Constraint;
    }
}
