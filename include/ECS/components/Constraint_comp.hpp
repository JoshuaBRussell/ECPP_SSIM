#pragma once

enum class CONSTR_TYPE {
    ROTATION 
};

struct Constraint_Component {
    
    int entity_id;

    CONSTR_TYPE constr_type;
};
