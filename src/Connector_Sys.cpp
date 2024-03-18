#include "Constraint.hpp"

#include "ECSManager.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "./components/Connector_comp.hpp"
#include "./components/Position_comp.hpp"
#include "./components/Velocity_comp.hpp"
#include "./components/Force_comp.hpp"
#include "./components/Torque_comp.hpp"
#include "./components/Rotation_comp.hpp"
#include "Eigen/src/Geometry/Rotation2D.h"

void Connector_System_Init(ECS_Manager &world);

void Connector_System(ECS_Manager &world){
    
    // Assume that gravity is a thing  
    for (auto it = world.get_component_begin<Connector_Component>(); 
              it < world.get_component_end<Connector_Component>(); it++){
        
        Eigen::Vector2f conn_body_pos = it->pos; // Location of Connector Relative to Body CoG 
        Eigen::Vector2f conn_force = it->force;  // Force Applied at Connector Body 
        float conn_torque = it->torque;  
   
        int attached_entity = it->attached_entity; // The Entity ID of the entity this connector is 
                                                   // attached to

        Force_Component* force_comp_ptr = world.get_component<Force_Component>(attached_entity);
        Torque_Component* torque_comp_ptr = world.get_component<Torque_Component>(attached_entity);
        Rotation_Component* rot_comp_ptr = world.get_component<Rotation_Component>(attached_entity);
        
        // Transfer the connection force to the CoG
        force_comp_ptr->force += conn_force;

        // Transform force into body space from world space
        Eigen::Rotation2D<float> transform_matr = Eigen::Rotation2D<float>((-3.14159/180.0)*rot_comp_ptr->angle);
        // Keeping this stale comment in b/c the fact that 'toRotatoinMatrix()' is needed is not 
        // intuitively obvious.
        // std::cout << transform_matr.toRotationMatrix() << std::endl;  
        Eigen::Vector2f conn_force_body = transform_matr * conn_force; 
        
        // Positive Torque is CCW
        float torque = -(conn_force_body.x() * conn_body_pos.y() - conn_force_body.y() * conn_body_pos.x()); 
        torque_comp_ptr->torque += torque;
    
    }

}

