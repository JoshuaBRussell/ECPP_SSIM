#include "Motion.hpp"

#include <algorithm>

#include "ECSManager.hpp"

#include <Eigen/Core>
#include <Eigen/LU> // inverse()
#include <Eigen/Dense>

#include "./components/Constraint_comp.hpp"
#include "./components/Position_comp.hpp"
#include "./components/Velocity_comp.hpp"
#include "./components/Acceleration_comp.hpp"
#include "./components/Force_comp.hpp"

size_t ENTITY_DIM = 2;

struct constr_info {
    int i; // constraint index;
    int j; // particle index

    float J_sub_block[2] = {0.0};
    float J_dot_sub_block[2] = {0.0};
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

void Constraint_System(ECS_Manager &world){
    
    // Assume that gravity is a thing  
    for (auto it = world.get_component_begin<Force_Component>(); 
              it < world.get_component_end<Force_Component>(); it++){
         
        Force_Component* force_comp_ptr = world.get_component<Force_Component>(it->entity_id); 
        force_comp_ptr->force = Eigen::Vector2f(0.0, -0.81); // g = 9.81 m/s^2 | m = 1 
    
    }
   
    // Need to have some way to ascribe entity locations in the global matrices,
    // and keep up with them in case the entity is encountered again in another 
    // constraint
    // This just used the index
    std::vector<int> constr_entities;
    std::vector<constr_info> constrs_vec;
     
    int constr_count = 0;
    
    // Collect info needed for each constraint
    for (auto it = world.get_component_begin<Fixed_Rot_Component>(); 
              it < world.get_component_end<Fixed_Rot_Component>(); it++){  
          
        int constr_entity = it->constr_entity;
        // Add Unique Item to List
        int entity_offset = 2*add_id_if_unique(&constr_entities, constr_entity); 
        
        
        Position_Component* pos_comp_ptr = world.get_component<Position_Component>(it->constr_entity);
        Velocity_Component* vel_comp_ptr = world.get_component<Velocity_Component>(it->constr_entity); 
        
        struct constr_info constr_info;
        constr_info.i              = constr_count;
        constr_info.j              = entity_offset;
        constr_info.J_sub_block[0] = 2.0*pos_comp_ptr->position.x();
        constr_info.J_sub_block[1] = 2.0*pos_comp_ptr->position.y(); 
        constr_info.J_dot_sub_block[0] = 2.0*vel_comp_ptr->velocity.x(); 
        constr_info.J_dot_sub_block[1] = 2.0*vel_comp_ptr->velocity.y();
        
        constrs_vec.push_back(constr_info);
   
        constr_count++;
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
        constr_info1.i              = constr_count;
        constr_info1.j              = constr_entity_offset1;
        constr_info1.J_sub_block[0] = -2.0*(pos_comp_ptr2->position.x() - pos_comp_ptr1->position.x());
        constr_info1.J_sub_block[1] = -2.0*(pos_comp_ptr2->position.y() - pos_comp_ptr1->position.y()); 
        constr_info1.J_dot_sub_block[0] = 2.0*(vel_comp_ptr1->velocity.x() - vel_comp_ptr2->velocity.x()); 
        constr_info1.J_dot_sub_block[1] = 2.0*(vel_comp_ptr1->velocity.y() - vel_comp_ptr2->velocity.y());
        
        constrs_vec.push_back(constr_info1); 
        
        struct constr_info constr_info2;
        constr_info2.i              = constr_count;
        constr_info2.j              = constr_entity_offset2;
        constr_info2.J_sub_block[0] = 2.0*(pos_comp_ptr2->position.x() - pos_comp_ptr1->position.x());
        constr_info2.J_sub_block[1] = 2.0*(pos_comp_ptr2->position.y() - pos_comp_ptr1->position.y()); 
        constr_info2.J_dot_sub_block[0] = -2.0*(vel_comp_ptr1->velocity.x() - vel_comp_ptr2->velocity.x()); 
        constr_info2.J_dot_sub_block[1] = -2.0*(vel_comp_ptr1->velocity.y() - vel_comp_ptr2->velocity.y());
        
        constrs_vec.push_back(constr_info2);
        constr_count++;
        
    }
    
    // ---- Form Global Matrices/Vectors ---- //
   
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> J(constr_count, ENTITY_DIM*constr_entities.size()); 
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> J_dot(constr_count, ENTITY_DIM*constr_entities.size()); 
    
    J = Eigen::MatrixXf::Zero(constr_count, ENTITY_DIM*constr_entities.size());
    J_dot = Eigen::MatrixXf::Zero(constr_count, ENTITY_DIM*constr_entities.size());

    std::cout << "Constraint Count: " << constr_count << std::endl;
    std::cout << "Constrained Entities " << constr_entities.size() << std::endl;
    
    for (auto it = constrs_vec.begin(); it < constrs_vec.end();  it++){
        std::cout << it->i << std::endl;
        std::cout << it->j << std::endl;
        std::cout << it->J_sub_block[0] << " | " << it->J_sub_block[1] << std::endl;
        std::cout << it->J_dot_sub_block[0] << " | " << it->J_dot_sub_block[1] << std::endl;

        J(it->i, it->j)   = it->J_sub_block[0]; 
        J(it->i, it->j+1) = it->J_sub_block[1];

        J_dot(it->i, it->j)   = it->J_dot_sub_block[0]; 
        J_dot(it->i, it->j+1) = it->J_dot_sub_block[1]; 
    }

    
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> M;
    M = Eigen::MatrixXf::Identity(ENTITY_DIM*constr_entities.size(), ENTITY_DIM*constr_entities.size());
    
    Eigen::VectorXf q_dot(ENTITY_DIM*constr_entities.size());
    Eigen::VectorXf Q(ENTITY_DIM*constr_entities.size());
    
    for (auto it = constr_entities.begin(); it < constr_entities.end(); it++){
        int entity_offset = 2*std::distance(constr_entities.begin(), it);
        
        Velocity_Component* vel_comp_ptr = world.get_component<Velocity_Component>(*it);  
        Force_Component* force_comp_ptr = world.get_component<Force_Component>(*it); 
        
        q_dot(entity_offset    ) = vel_comp_ptr->velocity.x(); 
        q_dot(entity_offset + 1) = vel_comp_ptr->velocity.y();

        Q(entity_offset    ) = force_comp_ptr->force.x(); 
        Q(entity_offset + 1) = force_comp_ptr->force.y();
        std::cout << "Fy" << std::endl;
        std::cout << Q(entity_offset + 1) << std::endl;

    }
    std::cout << "J: " << std::endl;
    std::cout << J << std::endl;

    // Solve Global Matrices
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> A = J*M.inverse()*J.transpose();
    Eigen::VectorXf b = -1.0*J_dot*q_dot - J*M.inverse()*Q;//- 2.5*C;
    
    Eigen::VectorXf x = A.fullPivHouseholderQr().solve(b);
    
    //\hat{Q}  = J^T\lambda

    Eigen::VectorXf Q_hat = J.transpose()*x;    

    // Apply Constraint Forces
    for (auto it = constr_entities.begin(); it < constr_entities.end(); it++){
        int entity_offset = 2*std::distance(constr_entities.begin(), it);
        
        Force_Component* force_comp_ptr = world.get_component<Force_Component>(*it); 
        force_comp_ptr->force.x() = force_comp_ptr->force.x() + Q_hat(entity_offset    ); 
        force_comp_ptr->force.y() = force_comp_ptr->force.y() + Q_hat(entity_offset + 1);
        std::cout << "Actual Fy" << std::endl;
        std::cout << force_comp_ptr->force.y() << std::endl; 
    }


/*
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
*/   
}
