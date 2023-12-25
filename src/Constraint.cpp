#include "Motion.hpp"

#include "ECSManager.hpp"

#include <Eigen/Core>
#include <Eigen/LU> // inverse()
#include <Eigen/Dense>

#include "./components/Constraint_comp.hpp"
#include "./components/Position_comp.hpp"
#include "./components/Velocity_comp.hpp"
#include "./components/Acceleration_comp.hpp"
#include "./components/Force_comp.hpp"

void Constraint_System(ECS_Manager &world){

    /*for (auto it = world.get_component_begin<Constraint_Component>(); 
              it < world.get_component_end<Constraint_Component>(); it++){ 
        
        Position_Component* pos_comp_ptr = world.get_component<Position_Component>(it->entity_id);
        Velocity_Component* vel_comp_ptr = world.get_component<Velocity_Component>(it->entity_id); 
        Force_Component* force_comp_ptr = world.get_component<Force_Component>(it->entity_id);
        
        // Find Constraint Force
        // Assume that gravity is a thing
        Eigen::Vector2f Force_App = Eigen::Vector2f(0.0, -0.81); // g = 9.81 m/s^2 | m = 1
        
        force_comp_ptr->force = Force_App + Force_Constraint;
    }*/

    // Had to switch order due to order obtained not be what was initially expected 
    auto it2 = world.get_component_begin<Constraint_Component>(); 
    auto it1 = it2+1; 
    
    Position_Component* pos_comp_ptr = world.get_component<Position_Component>(it1->entity_id);
    Velocity_Component* vel_comp_ptr = world.get_component<Velocity_Component>(it1->entity_id); 
    Force_Component* force_comp_ptr = world.get_component<Force_Component>(it1->entity_id);
    
    
    Position_Component* pos_comp_ptr2 = world.get_component<Position_Component>(it2->entity_id);
    Velocity_Component* vel_comp_ptr2 = world.get_component<Velocity_Component>(it2->entity_id); 
    Force_Component* force_comp_ptr2 = world.get_component<Force_Component>(it2->entity_id);


    // Find Constraint Force
    // Assume that gravity is a thing
    Eigen::Vector2f Force_App = Eigen::Vector2f(0.0, -0.81); // g = 9.81 m/s^2 | m = 1
    // JWJ^T\lambda = -\dot{J}\dot{q} - JWQ
    Eigen::Vector2f Force_Constraint = Eigen::Vector2f(0.0, 0.0);
    
    Eigen::Matrix<float, 2, 1> C = Eigen::MatrixXf::Zero(2, 1);

    C(0) = pos_comp_ptr->position.squaredNorm() - 1.0;
    C(1) = (pos_comp_ptr2->position - pos_comp_ptr->position).squaredNorm() - 1.0;

    Eigen::Matrix<float, 2, 4> J = Eigen::MatrixXf::Zero(2, 4);
    
    J(0, 0) = 2.0*pos_comp_ptr->position.x(); 
    J(0, 1) = 2.0*pos_comp_ptr->position.y(); 

    J(1, 0) = -2.0*(pos_comp_ptr2->position.x() - pos_comp_ptr->position.x());
    J(1, 1) = -2.0*(pos_comp_ptr2->position.y() - pos_comp_ptr->position.y()); 
    J(1, 2) =  2.0*(pos_comp_ptr2->position.x() - pos_comp_ptr->position.x()); 
    J(1, 3) =  2.0*(pos_comp_ptr2->position.y() - pos_comp_ptr->position.y()); 
    
    Eigen::Matrix<float, 4, 4> M = Eigen::Matrix<float, 4, 4>::Identity();
    
    Eigen::Matrix<float, 2, 4> J_dot= Eigen::MatrixXf::Zero(2, 4);
    
    J_dot(0, 0) = 2.0*vel_comp_ptr->velocity.x(); 
    J_dot(0, 1) = 2.0*vel_comp_ptr->velocity.y(); 
    
    J_dot(1, 0) = 2.0*(vel_comp_ptr->velocity.x() - vel_comp_ptr2->velocity.x());
    J_dot(1, 1) = 2.0*(vel_comp_ptr->velocity.y() - vel_comp_ptr2->velocity.y());
    J_dot(1, 2) = 2.0*(vel_comp_ptr2->velocity.x() - vel_comp_ptr->velocity.x());
    J_dot(1, 3) = 2.0*(vel_comp_ptr2->velocity.y() - vel_comp_ptr->velocity.y());
    
    Eigen::Vector4f q_dot;
    
    q_dot(0) = vel_comp_ptr->velocity.x();
    q_dot(1) = vel_comp_ptr->velocity.y();
    q_dot(2) = vel_comp_ptr2->velocity.x();
    q_dot(3) = vel_comp_ptr2->velocity.y();
    

    Eigen::Vector4f Q;
    Q(0) = Force_App.x(); 
    Q(1) = Force_App.y();
    Q(2) = Force_App.x();
    Q(3) = Force_App.y();
    
    Eigen::Matrix<float, 2, 2> A = J*M.inverse()*J.transpose();
    Eigen::Vector2f b = -1.0*J_dot*q_dot - J*M.inverse()*Q;//- 2.5*C;
    
    Eigen::Vector2f x = A.fullPivHouseholderQr().solve(b);
    
    //\hat{Q}  = J^T\lambda

    Eigen::Vector4f Q_hat = J.transpose()*x;

    force_comp_ptr->force  = Force_App + Eigen::Vector2f(Q_hat(0), Q_hat(1));
    force_comp_ptr2->force = Force_App + Eigen::Vector2f(Q_hat(2), Q_hat(3));
    
}
