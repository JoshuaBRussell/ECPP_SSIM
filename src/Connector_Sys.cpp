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
        
        Eigen::Vector2f conn_pos = it->pos; 
        Eigen::Vector2f conn_force = it->force; 
        Eigen::Vector2f conn_torque = it->torque;  
   
        int attached_entity = it->attached_entity;

        Force_Component* force_comp_ptr = world.get_component<Force_Component>(attached_entity);
        Torque_Component* torque_comp_ptr = world.get_component<Torque_Component>(attached_entity);
        Rotation_Component* rot_comp_ptr = world.get_component<Rotation_Component>(attached_entity);

        force_comp_ptr->force += conn_force;

        // Transform force into body space from world space
        Eigen::Rotation2D<float> transform_matr = Eigen::Rotation2D<float>((3.14159/180.0)*rot_comp_ptr->angle);

        // Converting this to a 3D Cross Product Problem since it is mentally easier
        conn_force = transform_matr * conn_force; 
        Eigen::Vector3f force_3d = Eigen::Vector3f();
        force_3d(0) = conn_force.x();
        force_3d(1) = conn_force.y(); 
        force_3d(2) = 0.0;
        
        Eigen::Vector3f pos_3d = Eigen::Vector3f();
        pos_3d(0) = conn_pos.x();
        pos_3d(1) = conn_pos.y(); 
        pos_3d(2) = 0.0;
        std::cout << "Value: \n";
        std::cout << pos_3d.cross(force_3d).norm() << std::endl;
        torque_comp_ptr->torque += pos_3d.cross(force_3d).norm(); 

    }

}

