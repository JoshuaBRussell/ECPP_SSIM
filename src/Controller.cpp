#include "Collision_comp.hpp"

#include <Eigen/Core>

#include "ECSManager.hpp"

#include "./components/Controller_comp.hpp"
#include "./components/Position_comp.hpp"
#include "./components/Acceleration_comp.hpp"

void Controller_System(ECS_Manager &world){ 
    
    //Assumes that the input is the Acceleration of the Component
    for (auto it = world.get_component_begin<Controller_Component>();
              it < world.get_component_end<Controller_Component>(); it++){
        
        Eigen::Vector2f des_pos = it->input;

        Eigen::Vector2f act_pos = world.get_component<Position_Component>(it->entity_id)->position;
        world.get_component<Acceleration_Component>(it->entity_id)->accel = it->K_Gain*(des_pos - act_pos);
    
    }
}

