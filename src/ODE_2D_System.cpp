#include "ODE_2D_System.hpp"

#include "ECSManager.hpp"

#include <string>

#include <Eigen/Core>

#include "./components/Position_comp.hpp"
#include "./components/ODE_2D_comp.hpp"


void ODE_System_2D_init(){

}

void ODE_System_2D(ECS_Manager &world, float dt){

    for (auto it = world.get_component_begin<ODE_2D_Component>(); 
              it < world.get_component_end<ODE_2D_Component>(); it++){ 
        
        Position_Component* pos_comp_ptr = world.get_component<Position_Component>(it->entity_id);
        
        ODE_2D_Component* ode_comp_ptr = world.get_component<ODE_2D_Component>(it->entity_id);  
        INT_METHOD method = ode_comp_ptr->integration_method; 

        Eigen::Vector2f input = Eigen::Vector2f();

        switch (method) {
            case INT_METHOD::EULER:{ 
                
                // Euler Method 
                pos_comp_ptr->position += dt*ODE_Function(pos_comp_ptr->position, input);
                break; 
                }
                 
            case INT_METHOD::RK4: {

                // Runge-Kutta | 4th Order
                Eigen::Vector2f K1 = ODE_Function(pos_comp_ptr->position, input);
                Eigen::Vector2f K2 = ODE_Function(pos_comp_ptr->position + (dt/2)*K1, input);
                Eigen::Vector2f K3 = ODE_Function(pos_comp_ptr->position + (dt/2)*K2, input);
                Eigen::Vector2f K4 = ODE_Function(pos_comp_ptr->position + (dt)*K3, input);
                pos_comp_ptr->position += (dt/6)*(K1 + 2*K2 + 2*K3 + K4, input);        
                break; 
                }
            
            default: {
                
                // Euler Method  
                pos_comp_ptr->position += dt*(*ODE_Function)(pos_comp_ptr->position, input);
                break;

                }
        } 
    }
}
