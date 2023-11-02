#pragma once

#include <string>

struct Render_Component {
    
    int entity_id;

    std::string texture_loc;
    int x;
    int y;
    int height;
    int width;
};
