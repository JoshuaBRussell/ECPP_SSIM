#include "Newtonian_Constraint_Sys.hpp"

#include "ECSManager.hpp"

#include <string>

#include "Vector2D.hpp"
#include "Vector.hpp"

#include "./components/Position_comp.hpp"
#include "./components/Velocity_comp.hpp"

#include "./components/ODE_comp.hpp"


void Newtonian_System_init(Vector2D (*f)(Vector2D)){

}

void Newtonian_Constraint_System(ECS_Manager &world, float dt){

    for (auto it = world.get_component_begin<ODE_Component>(); 
              it < world.get_component_end<ODE_Component>(); it++){ 
        
        Position_Component* pos_comp_ptr = world.get_component<Position_Component>(it->entity_id);
        Velocity_Component* vel_comp_ptr = world.get_component<Velocity_Component>(it->entity_id); 
        
        ODE_Component* ode_comp_ptr = world.get_component<ODE_Component>(it->entity_id);  
        INT_METHOD method = ode_comp_ptr->integration_method; 
        Vector<4> (*ODE_Function)(Vector<4>, Vector2D) = ode_comp_ptr->ODE_Function;

        // Find Constraint Force
        // Assume that gravity is a thing
        Vector2D Force_App = Vector2D(0.0, -0.81); // g = 9.81 m/s^2 | m = 1
        
        Vector2D pos_vec = pos_comp_ptr->position;
        Vector2D vel_vec = vel_comp_ptr->velocity;

        double lambda_not = (-(vel_vec.x*vel_vec.x + vel_vec.y*vel_vec.y) - (pos_vec.x*Force_App.x + pos_vec.y*Force_App.y));
        double lambda = lambda_not/(pos_vec.x*pos_vec.x +pos_vec.y*pos_vec.y);

        Vector2D Force_Constraint = lambda * pos_vec;

        Vector2D Force_Net = Force_App; //+ Force_Constraint;
        Vector2D acc_net = Force_Net; // m = 1 

        switch (method) {
            case INT_METHOD::EULER:{ 
                
                // Euler Method
                Vector<4> state_vec;
                state_vec[0] = pos_vec.x;
                state_vec[1] = vel_vec.x; 
                state_vec[2] = pos_vec.y; 
                state_vec[3] = vel_vec.y;
                
                state_vec = state_vec + dt*(*ODE_Function)(state_vec, acc_net);
                
                pos_comp_ptr->position = Vector2D(state_vec[0], state_vec[2]);
                vel_comp_ptr->velocity = Vector2D(state_vec[1], state_vec[3]);
                
                break; 
                }
                 
            case INT_METHOD::RK4: {

                // Runge-Kutta | 4th Order
                std::cout << "Not Implemented" << std::endl; 
                //Vector4D state_vec;
                //Vector2D K1 = ODE_Function(state_vec, acc_net);
                //Vector2D K2 = ODE_Function(pos_comp_ptr->position + (dt/2)*K1);
                //Vector2D K3 = ODE_Function(pos_comp_ptr->position + (dt/2)*K2);
                //Vector2D K4 = ODE_Function(pos_comp_ptr->position + (dt)*K3);
                //pos_comp_ptr->position += (dt/6)*(K1 + 2*K2 + 2*K3 + K4);        
                break; 
                }
            
            default: {
                
                // Euler Method  
                //pos_comp_ptr->position += dt*(*ODE_Function)(pos_comp_ptr->position);
                break;

                }
        } 
    }
}
