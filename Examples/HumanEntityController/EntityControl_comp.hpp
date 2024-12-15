#pragma once

typedef enum {
    NO_CMD     = 0, 
    UP_COMM    = 1,
    DOWN_COMM  = 2,
    LEFT_COMM  = 3,
    RIGHT_COMM = 4
} CommandDirections;

struct EntityControl_Component {
    
    int entity_id;

    CommandDirections comm_dir;

};
