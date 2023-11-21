#include "ODE_2D_System.hpp"

#include "ECSManager.hpp"

#include <string>

#include "Vector2D.hpp"

#include "./components/Position_comp.hpp"
#include "./components/ODE_2D_comp.hpp"


void ODE_System_2D_init(Vector2D (*f)(Vector2D)){

}

void ODE_System_2D(ECS_Manager &world, float dt){

    for (auto it = world.get_component_begin<ODE_2D_Component>(); 
              it < world.get_component_end<ODE_2D_Component>(); it++){ 
        
        Position_Component* pos_comp_ptr = world.get_component<Position_Component>(it->entity_id);
        
        ODE_2D_Component* ode_comp_ptr = world.get_component<ODE_2D_Component>(it->entity_id);  
        INT_METHOD method = ode_comp_ptr->integration_method; 
        Vector2D (*ODE_Function)(Vector2D, Vector2D) = ode_comp_ptr->ODE_Function;

        Vector2D input = Vector2D();

        switch (method) {
            case INT_METHOD::EULER:{ 
                
                // Euler Method 
                pos_comp_ptr->position += dt*(*ODE_Function)(pos_comp_ptr->position, input);
                break; 
                }
                 
            case INT_METHOD::RK4: {

                // Runge-Kutta | 4th Order
                Vector2D K1 = ODE_Function(pos_comp_ptr->position, input);
                Vector2D K2 = ODE_Function(pos_comp_ptr->position + (dt/2)*K1, input);
                Vector2D K3 = ODE_Function(pos_comp_ptr->position + (dt/2)*K2, input);
                Vector2D K4 = ODE_Function(pos_comp_ptr->position + (dt)*K3, input);
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
