#pragma once

#include <Eigen/Core>

struct Fixed_Rot_Component {
    
    int entity_id; 

    int constr_entity; 
    Eigen::Vector2f fixed_point;

};

struct Relative_Rot_Component {
    
    int entity_id;

    // The two entities to constrain
    int constr_entity1;
    int constr_entity2; 

};

