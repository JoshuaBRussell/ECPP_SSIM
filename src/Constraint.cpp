#include "Constraint.hpp"

#include <algorithm>
#include <cmath>

#include "ECSManager.hpp"

#include <Eigen/Core>
#include <Eigen/LU> // inverse()
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "./components/Constraint_comp.hpp"
#include "./components/Position_comp.hpp"
#include "./components/Velocity_comp.hpp"
#include "./components/Acceleration_comp.hpp"
#include "./components/Force_comp.hpp"
#include "./components/Torque_comp.hpp"
#include "./components/Rotation_comp.hpp"
#include "./components/Angular_Vel_comp.hpp"

const size_t CONSTR_DIM = 2; // Subject to change at later date
const size_t ENTITY_DIM = 3;
const double Kp_C = 25.0; 


struct constr_info {
    int i; // constraint index;
    int j; // particle index

    double J_sub_block[2][ENTITY_DIM] = {};
    double J_dot_sub_block[2][ENTITY_DIM] = {};
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

// Need to have some way to ascribe entity locations in the global matrices,
// and keep up with them in case the entity is encountered again in another 
// constraint
// This just used the index
static std::vector<int> constr_entities;
static std::vector<constr_info> constrs_vec;
static std::vector<double> constrs_eval;

static Eigen::SparseMatrix<double> A;
static Eigen::SparseMatrix<double> J; 
static Eigen::SparseMatrix<double> J_dot;
static Eigen::SparseMatrix<double> M;
static Eigen::SparseMatrix<double> W;

static Eigen::VectorXd q_dot;
static Eigen::VectorXd Q;
static Eigen::VectorXd C;


void Constraint_System_Init(ECS_Manager &world){
    
    // Hack to make sure that all components this system "needs"
    // doesn't crash in the instance that the user doesn't register the components
    world.register_component<Fixed_Rot_Component>();
    world.register_component<Linear_Component>();
    world.register_component<Relative_Rot_Component>(); 
    
    has_been_init = true;
    
    size_t constr_count = 0;
    // Go through all the currently list constraints to find the number of entities
    // involved.
    for (auto it = world.get_component_begin<Fixed_Rot_Component>(); 
              it < world.get_component_end<Fixed_Rot_Component>(); it++){
        add_id_if_unique(&constr_entities, it->constr_entity);
        constr_count +=1;
    }
    
    for (auto it = world.get_component_begin<Relative_Rot_Component>(); 
              it < world.get_component_end<Relative_Rot_Component>(); it++){
        add_id_if_unique(&constr_entities, it->constr_entity1); 
        add_id_if_unique(&constr_entities, it->constr_entity2); 
        constr_count+=1; 
    } 
    
    size_t entity_count = constr_entities.size();
    
    // Set total size of matrices
    J.resize(CONSTR_DIM*constr_count, ENTITY_DIM*entity_count);
    J_dot.resize(CONSTR_DIM*constr_count, ENTITY_DIM*entity_count); 
    // Reserve memory for non-zero elements 
    
    J.reserve(Eigen::VectorXd::Constant(ENTITY_DIM*entity_count, 4));
    J_dot.reserve(Eigen::VectorXd::Constant(ENTITY_DIM*entity_count, 4));

    // Only happenstance in this particular example
    M.resize(ENTITY_DIM*entity_count, ENTITY_DIM*entity_count);
    W.resize(ENTITY_DIM*entity_count, ENTITY_DIM*entity_count);
    M.setIdentity();
    W.setIdentity();

    q_dot.resize(ENTITY_DIM*entity_count, 1);
    Q.resize(ENTITY_DIM*entity_count, 1);
    C.resize(CONSTR_DIM*constr_count, 1);
    
    std::cout << "Constr Count: " << constr_count << std::endl;
    std::cout << "Entity Count: " << constr_count << std::endl;
    std::cout << "J rows: " << CONSTR_DIM*constr_count << std::endl;
    std::cout << "J cols: " << ENTITY_DIM*entity_count << std::endl;
}

void Constraint_System(ECS_Manager &world){
    
    // Clear these at the beginning to be sure they are empty
    // Empties the results, but keeps the capacity unchanged,
    // thereby reducing malloc calls under the hood
    constrs_vec.clear();
    constrs_eval.clear();

    // Check if init was called
    if (!has_been_init){
        std::cout << "WARNING: Constraint System has not been initialized. Call 'Constraint_init(<arg>)'" << std::endl;
    }

     
    
    // Collect info needed for each constraint
    for (auto it = world.get_component_begin<Fixed_Rot_Component>(); 
              it < world.get_component_end<Fixed_Rot_Component>(); it++){  
          
        // Find relative entity position 
        auto loc_it = std::find(constr_entities.begin(), constr_entities.end(), it->constr_entity);
        assert(loc_it != constr_entities.end()); // Either Init wasn't called OR Constraint_System was not notified of a new entity 
        size_t entity_offset = ENTITY_DIM*std::distance(constr_entities.begin(), loc_it); 
        
        
        Position_Component* pos_comp_ptr = world.get_component<Position_Component>(it->constr_entity);
        Velocity_Component* vel_comp_ptr = world.get_component<Velocity_Component>(it->constr_entity); 
        Rotation_Component* rot_comp_ptr = world.get_component<Rotation_Component>(it->constr_entity);
        Angular_Vel_Component* ang_vel_comp_ptr = world.get_component<Angular_Vel_Component>(it->constr_entity);
        
        // Convert the constrained body point position from body space to world space
        Eigen::Rotation2D<double> transform_matr = Eigen::Rotation2D<double>(rot_comp_ptr->angle);
        Eigen::Vector2d constr_body_pos = pos_comp_ptr->position + transform_matr * it->rel_body_pos;   
        
        //std::cout << "\nCoM Pos: \n";
        //std::cout << "X: " << pos_comp_ptr->position.x() << " Y: " << pos_comp_ptr->position.y();  
        //std::cout << "\nRot Pos: \n"; 
        //std::cout << "X: " << (transform_matr * it->rel_body_pos).x() << " Y: " << (transform_matr * it->rel_body_pos).y();  
        //std::cout << "\nNet Pos: \n"; 
        //std::cout << "X: " << constr_body_pos.x() << " Y: " << constr_body_pos.y();        
        
        struct constr_info constr_info;
        constr_info.i = constrs_eval.size();
        constr_info.j = entity_offset;
        
        double sin_theta = std::sin(rot_comp_ptr->angle);
        double cos_theta = std::cos(rot_comp_ptr->angle); 
        
        // Temp vars so I can get this working for now
        double x = pos_comp_ptr->position.x();
        double y = pos_comp_ptr->position.y();
        double x_dot = vel_comp_ptr->velocity.x();
        double y_dot = vel_comp_ptr->velocity.y(); 
        double rx = it->rel_body_pos.x(); 
        double ry = it->rel_body_pos.y();
        double theta = rot_comp_ptr->angle;
        double theta_dot = ang_vel_comp_ptr->w;
        

        constr_info.J_sub_block[0][0] = 1.0;
        constr_info.J_sub_block[0][1] = 0.0;
        constr_info.J_sub_block[0][2] = -rx*sin_theta - ry*cos_theta;

        constr_info.J_sub_block[1][0] = 0.0;
        constr_info.J_sub_block[1][1] = 1.0;
        constr_info.J_sub_block[1][2] = rx*cos_theta - ry*sin_theta; 
        
        Eigen::Vector2d r = transform_matr * it->rel_body_pos;
        Eigen::Vector2d temp_vec = ang_vel_comp_ptr->w*Eigen::Vector2d(-r.y(), r.x());// Contribution to World Space Vel due to Rotation is a Cross Product
        Eigen::Vector2d constr_body_vel = vel_comp_ptr->velocity + temp_vec; 

        /*std::cout << "\n\nCoM Vel: \n";
        std::cout << "X: " << vel_comp_ptr->velocity.x() << " Y: " << vel_comp_ptr->velocity.y();  
        std::cout << "\nRot Vel: \n"; 
        std::cout << "X: " << temp_vec.x() << " Y: " << temp_vec.y();  
        std::cout << "\nNet Vel: \n"; 
        std::cout << "X: " << constr_body_vel.x() << " Y: " << constr_body_vel.y() << "\n\n\n";  
        */
        constr_info.J_dot_sub_block[0][0] = 0.0; 
        constr_info.J_dot_sub_block[0][1] = 0.0;
        constr_info.J_dot_sub_block[0][2] = theta_dot*(-rx*cos_theta + ry*sin_theta); 
        
        constr_info.J_dot_sub_block[1][0] = 0.0; 
        constr_info.J_dot_sub_block[1][1] = 0.0;
        constr_info.J_dot_sub_block[1][2] = theta_dot*(-rx*sin_theta - ry*cos_theta); 
        
        constrs_vec.push_back(constr_info);
        //std::cout << "Fixed Rot Size: " << constrs_vec.size() << "\n";
   
        constrs_eval.push_back(constr_body_pos.x() - it->fixed_point.x());
        constrs_eval.push_back(constr_body_pos.y() - it->fixed_point.y()); 
    }
    
    for (auto it = world.get_component_begin<Relative_Rot_Component>(); 
              it < world.get_component_end<Relative_Rot_Component>(); it++){ 
        
        Rotation_Component* rot_comp_ptr1 = world.get_component<Rotation_Component>(it->constr_entity1);
        Angular_Vel_Component* ang_vel_comp_ptr1 = world.get_component<Angular_Vel_Component>(it->constr_entity1); 
         
        Rotation_Component* rot_comp_ptr2 = world.get_component<Rotation_Component>(it->constr_entity2);
        Angular_Vel_Component* ang_vel_comp_ptr2 = world.get_component<Angular_Vel_Component>(it->constr_entity2);
    
        double theta1 = rot_comp_ptr1->angle;
        double theta2 = rot_comp_ptr2->angle;

        double theta1_dot = ang_vel_comp_ptr1->w;
        double theta2_dot = ang_vel_comp_ptr2->w;
        
        
        double rx1 = it->rel_body_pos1.x(); 
        double ry1 = it->rel_body_pos1.y();
 
        double rx2 = it->rel_body_pos2.x(); 
        double ry2 = it->rel_body_pos2.y(); 

        double sin_theta1 = std::sin(theta1);
        double cos_theta1 = std::cos(theta1); 

        double sin_theta2 = std::sin(theta2);
        double cos_theta2 = std::cos(theta2);

                 
        
        // Constraint-Entity Pair #1
        
        // Find relative entity position 
        auto loc_it = std::find(constr_entities.begin(), constr_entities.end(), it->constr_entity1);
        assert(loc_it != constr_entities.end()); // Either Init wasn't called OR Constraint_System was not notified of a new entity
        size_t entity_offset1 = ENTITY_DIM*std::distance(constr_entities.begin(), loc_it);
        
        int constr_index = constrs_eval.size();   
        struct constr_info constr_info1;
        constr_info1.i = constr_index;
        constr_info1.j = entity_offset1; 
        
        // C_X_1 
        constr_info1.J_sub_block[0][0] = 1.0;
        constr_info1.J_sub_block[0][1] = 0.0;
        constr_info1.J_sub_block[0][2] = -rx1*sin_theta1 - ry1*cos_theta1;

        constr_info1.J_dot_sub_block[0][0] = 0.0; 
        constr_info1.J_dot_sub_block[0][1] = 0.0;
        constr_info1.J_dot_sub_block[0][2] = theta1_dot*(-rx1*cos_theta1 + ry1*sin_theta1); 

        // C_Y_1 
        constr_info1.J_sub_block[1][0] = 0.0;
        constr_info1.J_sub_block[1][1] = 1.0;
        constr_info1.J_sub_block[1][2] = rx1*cos_theta1 - ry1*sin_theta1; 
    
        constr_info1.J_dot_sub_block[1][0] = 0.0; 
        constr_info1.J_dot_sub_block[1][1] = 0.0;
        constr_info1.J_dot_sub_block[1][2] = theta1_dot*(-rx1*sin_theta1 - ry1*cos_theta1); 
        
        // Add the constraint related info to the vec
        constrs_vec.push_back(constr_info1);
        //std::cout << "Rel Rot Size#1: " << constrs_vec.size() << "\n"; 

        // Constraint-Entity Pair #2  
        loc_it = std::find(constr_entities.begin(), constr_entities.end(), it->constr_entity2);
        assert(loc_it != constr_entities.end()); // Either Init wasn't called OR Constraint_System was not notified of a new entity
        size_t entity_offset2 = ENTITY_DIM*std::distance(constr_entities.begin(), loc_it);        
        
        struct constr_info constr_info2;
        constr_info2.i = constrs_eval.size();
        constr_info2.j = entity_offset2; 
        
        // C_X_2 
        constr_info2.J_sub_block[0][0] = -1.0;
        constr_info2.J_sub_block[0][1] = 0.0;
        constr_info2.J_sub_block[0][2] = rx2*sin_theta2 + ry2*cos_theta2;

        constr_info2.J_dot_sub_block[0][0] = 0.0; 
        constr_info2.J_dot_sub_block[0][1] = 0.0;
        constr_info2.J_dot_sub_block[0][2] = theta2_dot*(rx2*cos_theta2 - ry2*sin_theta2); 

        // C_Y_2 
        constr_info2.J_sub_block[1][0] = 0.0;
        constr_info2.J_sub_block[1][1] = -1.0;
        constr_info2.J_sub_block[1][2] = -(rx2*cos_theta2 - ry2*sin_theta2); 
    
        constr_info2.J_dot_sub_block[1][0] = 0.0; 
        constr_info2.J_dot_sub_block[1][1] = 0.0;
        constr_info2.J_dot_sub_block[1][2] = theta2_dot*(rx2*sin_theta2 + ry2*cos_theta2);
        
        // Add the constraint related info to the vec 
        constrs_vec.push_back(constr_info2);
        //std::cout << "Rel Rot Size#2: " << constrs_vec.size() << "\n"; 
         
        // Evaluate and save the constaints
        Position_Component* pos_comp_ptr1 = world.get_component<Position_Component>(it->constr_entity1); 
        Position_Component* pos_comp_ptr2 = world.get_component<Position_Component>(it->constr_entity2); 

        double x1 = pos_comp_ptr1->position.x() + rx1*cos_theta1 - ry1*sin_theta1;  
        double x2 = pos_comp_ptr2->position.x() + rx2*cos_theta2 - ry2*sin_theta2;
        
        double y1 = pos_comp_ptr1->position.y() + rx1*sin_theta1 + ry1*cos_theta1;  
        double y2 = pos_comp_ptr2->position.y() + rx2*sin_theta2 + ry2*cos_theta2; 
        
        // Wait to add these values to the vectors so the size of the vector can indicate
        // where the constraint index is
        constrs_eval.push_back(x1 - x2);
        constrs_eval.push_back(y1 - y2);

    }

    // ---- Insert Values into the Jacobian Matrices/Vectors ---- //
    
    for (auto it = constrs_vec.begin(); it < constrs_vec.end();  it++){
        
        J.coeffRef(it->i  , it->j)   = it->J_sub_block[0][0]; 
        J.coeffRef(it->i  , it->j+1) = it->J_sub_block[0][1];
        J.coeffRef(it->i  , it->j+2) = it->J_sub_block[0][2];
        
        J.coeffRef(it->i+1, it->j)   = it->J_sub_block[1][0]; 
        J.coeffRef(it->i+1, it->j+1) = it->J_sub_block[1][1];
        J.coeffRef(it->i+1, it->j+2) = it->J_sub_block[1][2]; 
        
        
        J_dot.coeffRef(it->i  , it->j)   = it->J_dot_sub_block[0][0]; 
        J_dot.coeffRef(it->i  , it->j+1) = it->J_dot_sub_block[0][1]; 
        J_dot.coeffRef(it->i  , it->j+2) = it->J_dot_sub_block[0][2]; 
    
        J_dot.coeffRef(it->i+1, it->j)   = it->J_dot_sub_block[1][0]; 
        J_dot.coeffRef(it->i+1, it->j+1) = it->J_dot_sub_block[1][1]; 
        J_dot.coeffRef(it->i+1, it->j+2) = it->J_dot_sub_block[1][2];
    }
    
    for (auto it = constr_entities.begin(); it < constr_entities.end(); it++){
        int entity_offset = ENTITY_DIM*std::distance(constr_entities.begin(), it);
        
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
       //std::cout << "C: \n" << C << "\n";
    }
    
    
    // Solve Global Matrices
    A = J*W*J.transpose();
    Eigen::VectorXd b = -1.0*J_dot*q_dot - J*W*Q - Kp_C*C;
    //std::cout << "A: " << A << std::endl;
    //std::cout << "J: " << J << std::endl;
    //std::cout << "M: " << M << std::endl;
    //std::cout << "J_dot: " << J_dot << std::endl;
    //std::cout << "q_dot: " << q_dot << std::endl;
    //std::cout << "b: " << b << std::endl;
    //std::cout << "-1.0*J_dot*q_dot\n" << -1.0*J_dot*q_dot << "\n"; 
    //std::cout << "- J*M.inverse()*Q\n" << - J*M.inverse()*Q << "\n"; 
    //std::cout << "M.inverse()\n" << M.inverse() << "\n";
    //std::cout << "Q: \n" << Q << "\n";
    
    // Solver Methods
    Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<double> > lscg;   
    lscg.compute(A);
    Eigen::VectorXd x = lscg.solve(b);
    //Eigen::VectorXd x = A.fullPivHouseholderQr().solve(b);
    
    //\hat{Q}  = J^T\lambda
    Eigen::VectorXd Q_hat = J.transpose()*x;    
    //std::cout << "Q Hat: " << Q_hat << "\n";
    // Apply Constraint Forces
    for (auto it = constr_entities.begin(); it < constr_entities.end(); it++){
        int entity_offset = ENTITY_DIM*std::distance(constr_entities.begin(), it);
        //std::cout << "entity_offset: " << entity_offset << "\n";  
        // Apply the Forces
        Force_Component* force_comp_ptr = world.get_component<Force_Component>(*it); 
        force_comp_ptr->force.x() = force_comp_ptr->force.x() + Q_hat(entity_offset    ); 
        force_comp_ptr->force.y() = force_comp_ptr->force.y() + Q_hat(entity_offset + 1);
        
        // Apply the Torques
        // Positive Torque is CCW 
        Torque_Component* torque_comp_ptr = world.get_component<Torque_Component>(*it);
        torque_comp_ptr->torque += Q_hat(entity_offset + 2); 
        /* 
        //std::cout << "Constraint Force: \n";
        std::cout << "X: " << Q_hat(entity_offset) << " Y: " << Q_hat(entity_offset + 1) << std::endl;
        std::cout << "Net Force: \n"; 
        std::cout << "X: " << force_comp_ptr->force.x() << " Y: " << force_comp_ptr->force.y() << std::endl;
        std::cout << "Torque: \n";
        std::cout << torque_comp_ptr->torque << "\n" << std::endl;
        */ 
        
    }

}
