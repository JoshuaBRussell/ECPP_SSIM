#include "Constraint.hpp"

#include <algorithm>
#include <cmath>

#include "ECSManager.hpp"

#include <Eigen/Core>
#include <Eigen/LU> // inverse()
#include <Eigen/Dense>

#include "./components/Constraint_comp.hpp"
#include "./components/Position_comp.hpp"
#include "./components/Velocity_comp.hpp"
#include "./components/Acceleration_comp.hpp"
#include "./components/Force_comp.hpp"
#include "./components/Torque_comp.hpp"
#include "./components/Rotation_comp.hpp"
#include "./components/Angular_Vel_comp.hpp"

const size_t ENTITY_DIM = 3;
const float Kp_C = 0.0; 

struct constr_info {
    int i; // constraint index;
    int j; // particle index

    float J_sub_block[ENTITY_DIM] = {};
    float J_dot_sub_block[ENTITY_DIM] = {};
};

int add_id_if_unique(std::vector<int>* vec_ptr, int id){
    
    int return_index = -1;

    auto loc_it = std::find(vec_ptr->begin(), vec_ptr->end(), id);
    if (loc_it != vec_ptr->end()){ 
        return_index = std::distance(vec_ptr->begin(), loc_it);   
    } else {
        // Add it 
        vec_ptr->push_back(id);
    
        // Now find its index
        auto loc_it = std::find(vec_ptr->begin(), vec_ptr->end(), id);
        return_index = std::distance(vec_ptr->begin(), loc_it);   
    };

    return return_index;
}

bool has_been_init = false;

void Constraint_System_Init(ECS_Manager &world){
    
    // Hack to make sure that all components this system "needs"
    // doesn't crash in the instance that the user doesn't register the components
    world.register_component<Fixed_Rot_Component>();
    world.register_component<Linear_Component>();
    world.register_component<Relative_Rot_Component>(); 
    
    has_been_init = true;

}

void Constraint_System(ECS_Manager &world){
    

    // Check if init was called
    if (!has_been_init){
        std::cout << "WARNING: Constraint System has not been initialized. Call 'Constraint_init(<arg>)'" << std::endl;
    }

    // Need to have some way to ascribe entity locations in the global matrices,
    // and keep up with them in case the entity is encountered again in another 
    // constraint
    // This just used the index
    std::vector<int> constr_entities;
    std::vector<constr_info> constrs_vec;
    std::vector<float> constrs_eval; 
    
    // Collect info needed for each constraint
    for (auto it = world.get_component_begin<Fixed_Rot_Component>(); 
              it < world.get_component_end<Fixed_Rot_Component>(); it++){  
          
        int constr_entity = it->constr_entity;
        // Add Unique Item to List
        int entity_offset = 2*add_id_if_unique(&constr_entities, constr_entity); 
        
        
        Position_Component* pos_comp_ptr = world.get_component<Position_Component>(it->constr_entity);
        Velocity_Component* vel_comp_ptr = world.get_component<Velocity_Component>(it->constr_entity); 
        Rotation_Component* rot_comp_ptr = world.get_component<Rotation_Component>(it->constr_entity);
        Angular_Vel_Component* ang_vel_comp_ptr = world.get_component<Angular_Vel_Component>(it->constr_entity);
        
        // Convert the constrained body point position from body space to world space
        Eigen::Rotation2D<float> transform_matr = Eigen::Rotation2D<float>((3.14159/180.0)*rot_comp_ptr->angle);
        Eigen::Vector2f constr_body_pos = pos_comp_ptr->position + transform_matr * it->rel_body_pos;   
        
        std::cout << "\nCoM Pos: \n";
        std::cout << "X: " << pos_comp_ptr->position.x() << " Y: " << pos_comp_ptr->position.y();  
        std::cout << "\nRot Pos: \n"; 
        std::cout << "X: " << (transform_matr * it->rel_body_pos).x() << " Y: " << (transform_matr * it->rel_body_pos).y();  
        std::cout << "\nNet Pos: \n"; 
        std::cout << "X: " << constr_body_pos.x() << " Y: " << constr_body_pos.y();        
        
        struct constr_info constr_info;
        constr_info.i = constrs_eval.size();
        constr_info.j = entity_offset;
        
        constr_info.J_sub_block[0] = (constr_body_pos.x());
        constr_info.J_sub_block[1] = (constr_body_pos.y());
       
        float sin_theta = std::sin((3.14159/180.0)*rot_comp_ptr->angle);
        float cos_theta = std::cos((3.14159/180.0)*rot_comp_ptr->angle);
        float jac_angle_component = constr_body_pos.x()*(-1.0*it->rel_body_pos.x()*sin_theta
                                                                - it->rel_body_pos.y()*cos_theta)
                                  + constr_body_pos.y()*(it->rel_body_pos.x()*cos_theta
                                                           - it->rel_body_pos.y()*sin_theta);
        constr_info.J_sub_block[2] = jac_angle_component;

        Eigen::Vector2f r = transform_matr * it->rel_body_pos;
        Eigen::Vector2f temp_vec = ang_vel_comp_ptr->w*Eigen::Vector2f(-r.y(), r.x());// Contribution to World Space Vel due to Rotation is a Cross Product
        Eigen::Vector2f constr_body_vel = vel_comp_ptr->velocity + temp_vec; 

        std::cout << "\n\nCoM Vel: \n";
        std::cout << "X: " << vel_comp_ptr->velocity.x() << " Y: " << vel_comp_ptr->velocity.y();  
        std::cout << "\nRot Vel: \n"; 
        std::cout << "X: " << temp_vec.x() << " Y: " << temp_vec.y();  
        std::cout << "\nNet Vel: \n"; 
        std::cout << "X: " << constr_body_vel.x() << " Y: " << constr_body_vel.y() << "\n\n\n";  
        
        // Temp vars so I can get this working for now
        float x = pos_comp_ptr->position.x();
        float y = pos_comp_ptr->position.y();
        float x_dot = vel_comp_ptr->velocity.x();
        float y_dot = vel_comp_ptr->velocity.y(); 
        float rx = it->rel_body_pos.x(); 
        float ry = it->rel_body_pos.y();
        float theta = rot_comp_ptr->angle;
        float theta_dot = ang_vel_comp_ptr->w; 
        constr_info.J_dot_sub_block[0] = (x_dot + theta_dot*(-rx*sin_theta - ry*cos_theta)); 
        constr_info.J_dot_sub_block[1] = (y_dot + theta_dot*( rx*cos_theta - ry*sin_theta));
        
        constr_info.J_dot_sub_block[2] = theta_dot*(x*(ry*sin_theta - rx*cos_theta) - y*(rx*sin_theta + ry*cos_theta)); 
        
        constrs_vec.push_back(constr_info);
   
        //x^2 + y^2 - r^2
        float constr_val = 0.0;//(constr_body_pos).squaredNorm() - it->radius*it->radius; 
        constrs_eval.push_back(constr_val);
    }

    // Collect info needed for each constraint
    for (auto it = world.get_component_begin<Linear_Component>(); 
              it < world.get_component_end<Linear_Component>(); it++){  
          
        int constr_entity = it->constr_entity;
        // Add Unique Item to List
        int entity_offset = 2*add_id_if_unique(&constr_entities, constr_entity); 
        
        Position_Component* pos_comp_ptr = world.get_component<Position_Component>(it->constr_entity);
        
        // Copying into stack variables to make code easier to read
        float m = it->m;
        float b = it->b;
        float p_x = pos_comp_ptr->position.x();
        float p_y = pos_comp_ptr->position.y();

        struct constr_info constr_info;
        constr_info.i              = constrs_eval.size();
        constr_info.j              = entity_offset;
        constr_info.J_sub_block[0] = -2.0*m*(p_y - (m*p_x + b));
        constr_info.J_sub_block[1] =  2.0*  (p_y - (m*p_x+b));
        constr_info.J_dot_sub_block[0] = 2.0*(-2*m*p_y + 2*m*m*p_x + m*b); 
        constr_info.J_dot_sub_block[1] = 2.0*(-2*m*p_x + 2*p_y - b);
        
        constrs_vec.push_back(constr_info);
        //y_d  = (m*x +b)  
        //C(i) = |p - p_desired|
        //     = (x - x_d)^2 + (y - y_d)^2
        //     = (y - (m*x + b))^2 
        float constr_val = (p_y - (m*p_x + b)) * (p_y - (m*p_x + b)); 
        constrs_eval.push_back(constr_val);
    }

    // Collect info needed for each constraint
    for (auto it = world.get_component_begin<Relative_Rot_Component>(); 
              it < world.get_component_end<Relative_Rot_Component>(); it++){ 
        
        int constr_entity1 = it->constr_entity1;
        int constr_entity2 = it->constr_entity2;
        
        int constr_entity_offset1 = 2*add_id_if_unique(&constr_entities, constr_entity1);
        int constr_entity_offset2 = 2*add_id_if_unique(&constr_entities, constr_entity2);
        
        Position_Component* pos_comp_ptr1 = world.get_component<Position_Component>(it->constr_entity1);
        Velocity_Component* vel_comp_ptr1 = world.get_component<Velocity_Component>(it->constr_entity1); 

        Position_Component* pos_comp_ptr2 = world.get_component<Position_Component>(it->constr_entity2);
        Velocity_Component* vel_comp_ptr2 = world.get_component<Velocity_Component>(it->constr_entity2); 
        
        // For the first entity
        struct constr_info constr_info1;
        constr_info1.i              = constrs_eval.size();
        constr_info1.j              = constr_entity_offset1;
        constr_info1.J_sub_block[0] = -2.0*(pos_comp_ptr2->position.x() - pos_comp_ptr1->position.x());
        constr_info1.J_sub_block[1] = -2.0*(pos_comp_ptr2->position.y() - pos_comp_ptr1->position.y()); 
        constr_info1.J_dot_sub_block[0] = 2.0*(vel_comp_ptr1->velocity.x() - vel_comp_ptr2->velocity.x()); 
        constr_info1.J_dot_sub_block[1] = 2.0*(vel_comp_ptr1->velocity.y() - vel_comp_ptr2->velocity.y());
        
        constrs_vec.push_back(constr_info1); 
        
        struct constr_info constr_info2;
        constr_info2.i              = constrs_eval.size();
        constr_info2.j              = constr_entity_offset2;
        constr_info2.J_sub_block[0] = 2.0*(pos_comp_ptr2->position.x() - pos_comp_ptr1->position.x());
        constr_info2.J_sub_block[1] = 2.0*(pos_comp_ptr2->position.y() - pos_comp_ptr1->position.y()); 
        constr_info2.J_dot_sub_block[0] = -2.0*(vel_comp_ptr1->velocity.x() - vel_comp_ptr2->velocity.x()); 
        constr_info2.J_dot_sub_block[1] = -2.0*(vel_comp_ptr1->velocity.y() - vel_comp_ptr2->velocity.y());
        
        constrs_vec.push_back(constr_info2);

        //(x2 - x1)^2 + (y2 - y1)^2 - r^2
        float constr_val = (pos_comp_ptr2->position - pos_comp_ptr1->position).squaredNorm() - it->radius*it->radius; 
        constrs_eval.push_back(constr_val);
    }
    
    // ---- Form Global Matrices/Vectors ---- //
   
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> J(constrs_eval.size(), ENTITY_DIM*constr_entities.size()); 
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> J_dot(constrs_eval.size(), ENTITY_DIM*constr_entities.size()); 
    
    J = Eigen::MatrixXf::Zero(constrs_eval.size(), ENTITY_DIM*constr_entities.size());
    J_dot = Eigen::MatrixXf::Zero(constrs_eval.size(), ENTITY_DIM*constr_entities.size());

    
    for (auto it = constrs_vec.begin(); it < constrs_vec.end();  it++){

        J(it->i, it->j)   = it->J_sub_block[0]; 
        J(it->i, it->j+1) = it->J_sub_block[1];
        J(it->i, it->j+2) = it->J_sub_block[2];
        
        J_dot(it->i, it->j)   = it->J_dot_sub_block[0]; 
        J_dot(it->i, it->j+1) = it->J_dot_sub_block[1]; 
        J_dot(it->i, it->j+2) = it->J_dot_sub_block[2]; 
    }

    
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> M;
    M = Eigen::MatrixXf::Identity(ENTITY_DIM*constr_entities.size(), ENTITY_DIM*constr_entities.size());
    
    Eigen::VectorXf q_dot(ENTITY_DIM*constr_entities.size());
    Eigen::VectorXf Q(ENTITY_DIM*constr_entities.size());
    Eigen::VectorXf C(constrs_eval.size()); 
    
    for (auto it = constr_entities.begin(); it < constr_entities.end(); it++){
        int entity_offset = 2*std::distance(constr_entities.begin(), it);
        
        // Translational 
        Velocity_Component* vel_comp_ptr = world.get_component<Velocity_Component>(*it);  
        Force_Component* force_comp_ptr = world.get_component<Force_Component>(*it); 
        
        // Rotational
        Angular_Vel_Component* ang_vel_comp_ptr = world.get_component<Angular_Vel_Component>(*it); 
        Torque_Component* torque_comp_ptr = world.get_component<Torque_Component>(*it); 
        
        q_dot(entity_offset    ) = vel_comp_ptr->velocity.x(); 
        q_dot(entity_offset + 1) = vel_comp_ptr->velocity.y();
        q_dot(entity_offset + 2) = ang_vel_comp_ptr->w;
        
        Q(entity_offset    ) = force_comp_ptr->force.x(); 
        Q(entity_offset + 1) = force_comp_ptr->force.y();
        Q(entity_offset + 2) = torque_comp_ptr->torque;

    }

    // Copy the collected evaluated constraint functions into
    // this vector to be used when solving for the constraint
    // forces  
    for (size_t i = 0; i < constrs_eval.size(); i++){
       C(i) = constrs_eval[i]; 
    }

    // Solve Global Matrices
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> A = J*M.inverse()*J.transpose();
    Eigen::VectorXf b = -1.0*J_dot*q_dot - J*M.inverse()*Q - Kp_C*C;
    std::cout << "A: " << A << std::endl;
    std::cout << "J: " << J << std::endl;
    std::cout << "M: " << M << std::endl;
    std::cout << "J_dot: " << J_dot << std::endl;
    std::cout << "q_dot: " << q_dot << std::endl;
    std::cout << "b: " << b << std::endl; 
    Eigen::VectorXf x = A.fullPivHouseholderQr().solve(b);
    
    //\hat{Q}  = J^T\lambda
    Eigen::VectorXf Q_hat = J.transpose()*x;    

    // Apply Constraint Forces
    for (auto it = constr_entities.begin(); it < constr_entities.end(); it++){
        int entity_offset = 2*std::distance(constr_entities.begin(), it);
        
        // Apply the Forces
        Force_Component* force_comp_ptr = world.get_component<Force_Component>(*it); 
        force_comp_ptr->force.x() = force_comp_ptr->force.x() + Q_hat(entity_offset    ); 
        force_comp_ptr->force.y() = force_comp_ptr->force.y() + Q_hat(entity_offset + 1);
        
        // Apply the Torques
        // Positive Torque is CCW 
        Torque_Component* torque_comp_ptr = world.get_component<Torque_Component>(*it);
        torque_comp_ptr->torque += Q_hat(entity_offset + 2); 
         
        std::cout << "Constraint Force: \n";
        std::cout << "X: " << Q_hat(entity_offset) << " Y: " << Q_hat(entity_offset + 1) << std::endl;
        std::cout << "Net Force: \n"; 
        std::cout << "X: " << force_comp_ptr->force.x() << " Y: " << force_comp_ptr->force.y() << std::endl;
        std::cout << "Torque: \n";
        std::cout << torque_comp_ptr->torque << "\n" << std::endl;
    }

}
