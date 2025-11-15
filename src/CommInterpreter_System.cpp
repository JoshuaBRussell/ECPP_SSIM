#include <Eigen/Core>

#include "ECSManager.hpp"

#include "./../Examples/HumanEntityController/EntityControl_comp.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "Force_comp.hpp"

void CommInterpreter_System(ECS_Manager &world){ 
    
    //Assumes that the input is the Acceleration of the Component
    for (auto it = world.get_component_begin<EntityControl_Component>();
              it < world.get_component_end<EntityControl_Component>(); it++){
        
        Eigen::Vector2d comm_force = Eigen::Vector2d(0.0, 0.0);
        
        switch (it->comm_dir) {

            case CommandDirections::NO_CMD:
                // Do Nothing
            break;
        
            case CommandDirections::UP_COMM:
                comm_force = Eigen::Vector2d(+0.0, +1.0);
            break;

            case CommandDirections::DOWN_COMM:
                comm_force = Eigen::Vector2d(+0.0, -1.0);
            break;

            case CommandDirections::LEFT_COMM:
                comm_force = Eigen::Vector2d(-1.0, +0.0);
            break;
            
            case CommandDirections::RIGHT_COMM:
                comm_force = Eigen::Vector2d(+1.0, +0.0);
            break;

        }

        world.get_component<Force_Component>(it->entity_id)->force += comm_force;

        it->comm_dir = CommandDirections::NO_CMD;
        
    }
}

