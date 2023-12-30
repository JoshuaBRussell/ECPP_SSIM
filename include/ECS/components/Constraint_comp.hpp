#pragma once

#include <Eigen/Core>

struct Fixed_Rot_Component {
    
    int entity_id; 

    int constr_entity; 
    Eigen::Vector2f fixed_point;
    float radius;
};

struct Relative_Rot_Component {
    
    int entity_id;

    // The two entities to constrain
    int constr_entity1;
    int constr_entity2;

    float radius;

};

struct Linear_Component {
    
    int entity_id;

    int constr_entity;
    
    // Defines the line to be constrained to
    float m;
    float b;

};
