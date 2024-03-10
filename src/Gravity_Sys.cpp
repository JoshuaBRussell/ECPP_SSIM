#include "Gravity_Sys.hpp"

#include "ECSManager.hpp"

#include <Eigen/Core>

#include "./components/Gravity_comp.hpp"
#include "./components/Force_comp.hpp"
#include "./components/Mass_comp.hpp"
#include "Torque_comp.hpp"

#define GRAVITATIONAL_ACC -9.81 // m/s^2

void Gravity_System(ECS_Manager &world){

    for (auto it = world.get_component_begin<Gravity_Component>(); 
              it < world.get_component_end<Gravity_Component>(); it++){ 
        
        float grav_force = GRAVITATIONAL_ACC * world.get_component<Mass_Component>(it->entity_id)->m;

        world.get_component<Force_Component>(it->entity_id)->force += Eigen::Vector2f(0.0, grav_force);
    
    }
}
