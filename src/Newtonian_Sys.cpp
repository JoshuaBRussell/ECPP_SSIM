#include "Newtonian_Sys.hpp"

#include "ECSManager.hpp"

#include <string>
#include <Eigen/Core>

#include "./components/Position_comp.hpp"
#include "./components/Velocity_comp.hpp"
#include "./components/Rotation_comp.hpp"
#include "./components/Angular_Vel_comp.hpp"
#include "./components/Force_comp.hpp"
#include "./components/Torque_comp.hpp"
#include "./components/Mass_comp.hpp"
#include "./components/Rot_Inertia_comp.hpp"
#include "./components/ODE_comp.hpp"

// Newtonian ODE Function
// state: {pos_x, vel_x, pos_y, vel_y, theta, theta_dot}
// input: {acc_x, acc_y, theta_ddot}
Eigen::Matrix<float, 6, 1> ODE_Function(Eigen::Matrix<float, 6, 1> state, Eigen::Vector3f input){
    
    Eigen::Matrix<float, 6, 1> state_dot; // The derivative of state
    
    state_dot(0) = state.coeff(1);
    state_dot(1) = input.coeff(0); 
    state_dot(2) = state.coeff(3); 
    state_dot(3) = input.coeff(1);
    state_dot(4) = state.coeff(5);
    state_dot(5) = input.coeff(2);
    
    return state_dot;
}

void Newtonian_System_init(){

}

void Newtonian_System(ECS_Manager &world, float dt){

    for (auto it = world.get_component_begin<ODE_Component>(); 
              it < world.get_component_end<ODE_Component>(); it++){ 
       
        Force_Component* force_comp_ptr = world.get_component<Force_Component>(it->entity_id);
        Torque_Component* torque_comp_ptr = world.get_component<Torque_Component>(it->entity_id);
        
        Mass_Component* mass_comp_ptr = world.get_component<Mass_Component>(it->entity_id);        
        Rot_Inertia_Component* rot_inertia_comp_ptr = world.get_component<Rot_Inertia_Component>(it->entity_id);
        
        Position_Component* pos_comp_ptr = world.get_component<Position_Component>(it->entity_id);
        Velocity_Component* vel_comp_ptr = world.get_component<Velocity_Component>(it->entity_id); 
        Rotation_Component* rot_comp_ptr = world.get_component<Rotation_Component>(it->entity_id); 
        Angular_Vel_Component* ang_vel_comp_ptr = world.get_component<Angular_Vel_Component>(it->entity_id);
        
        ODE_Component* ode_comp_ptr = world.get_component<ODE_Component>(it->entity_id);  
        
        
        INT_METHOD method = ode_comp_ptr->integration_method;

        Eigen::Vector2f pos_vec = pos_comp_ptr->position;
        Eigen::Vector2f vel_vec = vel_comp_ptr->velocity;
        float theta = rot_comp_ptr->angle;
        float w = ang_vel_comp_ptr->w;
        
        Eigen::Vector2f lin_acc_net = force_comp_ptr->force / mass_comp_ptr->m;
        float ang_acc_net = torque_comp_ptr->torque / rot_inertia_comp_ptr->moment_of_inertia;

        switch (method) {
            case INT_METHOD::EULER:{ 
                
                // Euler Method
                Eigen::Matrix<float, 6, 1> state_vec;
                state_vec(0) = pos_vec.coeff(0);
                state_vec(1) = vel_vec.coeff(0); 
                state_vec(2) = pos_vec.coeff(1); 
                state_vec(3) = vel_vec.coeff(1);
                state_vec(4) = theta; 
                state_vec(5) = w;

                Eigen::Vector3f acc_net;
                acc_net(0) = lin_acc_net(0);
                acc_net(1) = lin_acc_net(1);
                acc_net(2) = ang_acc_net;
                
                state_vec = state_vec + dt*ODE_Function(state_vec, acc_net);
                
                pos_comp_ptr->position = Eigen::Vector2f(state_vec[0], state_vec[2]);
                vel_comp_ptr->velocity = Eigen::Vector2f(state_vec[1], state_vec[3]);
                rot_comp_ptr->angle    = state_vec[4];
                ang_vel_comp_ptr->w    = state_vec[5];

                break; 
                }
                 
            case INT_METHOD::RK4: {

                // Runge-Kutta | 4th Order
                Eigen::Matrix<float, 6, 1> state_vec;
                state_vec(0) = pos_vec.coeff(0);
                state_vec(1) = vel_vec.coeff(0); 
                state_vec(2) = pos_vec.coeff(1); 
                state_vec(3) = vel_vec.coeff(1);
                state_vec(4) = theta; 
                state_vec(5) = w;

                Eigen::Vector3f acc_net;
                acc_net(0) = lin_acc_net(0);
                acc_net(1) = lin_acc_net(1);
                acc_net(2) = ang_acc_net;
                
                Eigen::Matrix<float, 6, 1> K1 = ODE_Function(state_vec            , acc_net);
                Eigen::Matrix<float, 6, 1> K2 = ODE_Function(state_vec + (dt/2)*K1, acc_net);
                Eigen::Matrix<float, 6, 1> K3 = ODE_Function(state_vec + (dt/2)*K2, acc_net);
                Eigen::Matrix<float, 6, 1> K4 = ODE_Function(state_vec + (dt)*K3  , acc_net);
                
                state_vec = state_vec + (dt/6)*(K1 + 2*K2 + 2*K3 + K4);

                pos_comp_ptr->position = Eigen::Vector2f(state_vec[0], state_vec[2]);
                vel_comp_ptr->velocity = Eigen::Vector2f(state_vec[1], state_vec[3]);
                rot_comp_ptr->angle    = state_vec[4];
                ang_vel_comp_ptr->w    = state_vec[5]; 
                
                break; 
                }
            
            default: {
                
                // Euler Method
                Eigen::Matrix<float, 6, 1> state_vec;
                state_vec(0) = pos_vec.coeff(0);
                state_vec(1) = vel_vec.coeff(0); 
                state_vec(2) = pos_vec.coeff(1); 
                state_vec(3) = vel_vec.coeff(1);
                state_vec(4) = theta; 
                state_vec(5) = w;

                Eigen::Vector3f acc_net;
                acc_net(0) = lin_acc_net(0);
                acc_net(1) = lin_acc_net(1);
                acc_net(2) = ang_acc_net;
                
                state_vec = state_vec + dt*ODE_Function(state_vec, acc_net);
                
                pos_comp_ptr->position = Eigen::Vector2f(state_vec[0], state_vec[2]);
                vel_comp_ptr->velocity = Eigen::Vector2f(state_vec[1], state_vec[3]);
                rot_comp_ptr->angle    = state_vec[4];
                ang_vel_comp_ptr->w    = state_vec[5];
                break;
                }
        }

        force_comp_ptr->force = Eigen::Vector2f(0.0, 0.0);
        torque_comp_ptr->torque = 0.0; 
    }
}
