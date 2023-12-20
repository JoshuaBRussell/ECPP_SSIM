#include "Newtonian_Sys.hpp"

#include "ECSManager.hpp"

#include <string>
#include <Eigen/Core>

#include "./components/Position_comp.hpp"
#include "./components/Velocity_comp.hpp"
#include "./components/Force_comp.hpp"

#include "./components/ODE_comp.hpp"

Eigen::Vector4f ODE_Function(Eigen::Vector4f state, Eigen::Vector2f input){
    
    Eigen::Vector4f state_dot; // The derivative of state
    
    state_dot(0) = state.coeff(1);
    state_dot(1) = input.coeff(0); 
    state_dot(2) = state.coeff(3); 
    state_dot(3) = input.coeff(1);
    
    return state_dot;
}

void Newtonian_System_init(){

}

void Newtonian_System(ECS_Manager &world, float dt){

    for (auto it = world.get_component_begin<ODE_Component>(); 
              it < world.get_component_end<ODE_Component>(); it++){ 
        
        Position_Component* pos_comp_ptr = world.get_component<Position_Component>(it->entity_id);
        Velocity_Component* vel_comp_ptr = world.get_component<Velocity_Component>(it->entity_id); 
        Force_Component* force_comp_ptr = world.get_component<Force_Component>(it->entity_id);

        ODE_Component* ode_comp_ptr = world.get_component<ODE_Component>(it->entity_id);  
        
        
        INT_METHOD method = ode_comp_ptr->integration_method;

        Eigen::Vector2f pos_vec = pos_comp_ptr->position;
        Eigen::Vector2f vel_vec = vel_comp_ptr->velocity;

        Eigen::Vector2f acc_net = force_comp_ptr->force;// m = 1 

        switch (method) {
            case INT_METHOD::EULER:{ 
                
                // Euler Method
                Eigen::Vector4f state_vec;
                state_vec(0) = pos_vec.coeff(0);
                state_vec(1) = vel_vec.coeff(0); 
                state_vec(2) = pos_vec.coeff(1); 
                state_vec(3) = vel_vec.coeff(1);
                
                state_vec = state_vec + dt*ODE_Function(state_vec, acc_net);
                
                pos_comp_ptr->position = Eigen::Vector2f(state_vec[0], state_vec[2]);
                vel_comp_ptr->velocity = Eigen::Vector2f(state_vec[1], state_vec[3]);
                
                break; 
                }
                 
            case INT_METHOD::RK4: {

                // Runge-Kutta | 4th Order
                Eigen::Vector4f state_vec;
                state_vec(0) = pos_vec.coeff(0);
                state_vec(1) = vel_vec.coeff(0); 
                state_vec(2) = pos_vec.coeff(1); 
                state_vec(3) = vel_vec.coeff(1);
                
                Eigen::Vector4f K1 = ODE_Function(state_vec            , acc_net);
                Eigen::Vector4f K2 = ODE_Function(state_vec + (dt/2)*K1, acc_net);
                Eigen::Vector4f K3 = ODE_Function(state_vec + (dt/2)*K2, acc_net);
                Eigen::Vector4f K4 = ODE_Function(state_vec + (dt)*K3  , acc_net);
                
                state_vec = state_vec + (dt/6)*(K1 + 2*K2 + 2*K3 + K4);

                pos_comp_ptr->position = Eigen::Vector2f(state_vec[0], state_vec[2]);
                vel_comp_ptr->velocity = Eigen::Vector2f(state_vec[1], state_vec[3]);
                
                break; 
                }
            
            default: {
                
                // Euler Method
                Eigen::Vector4f state_vec;
                state_vec[0] = pos_vec(0);
                state_vec[1] = vel_vec(0); 
                state_vec[2] = pos_vec(1); 
                state_vec[3] = vel_vec(1);
                
                state_vec = state_vec + dt*(*ODE_Function)(state_vec, acc_net);
                
                pos_comp_ptr->position = Eigen::Vector2f(state_vec[0], state_vec[2]);
                vel_comp_ptr->velocity = Eigen::Vector2f(state_vec[1], state_vec[3]);

                break;
                }
        } 
    }
}
