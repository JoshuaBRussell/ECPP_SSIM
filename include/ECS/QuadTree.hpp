#pragma once

#include "ECSManager.hpp"
#include "Vector2D.hpp"


struct DataNode {
    int ID;

    struct DataNode *next;
};

struct QuadNode {
    
    //Children (if existent)
    struct QuadNode *first_child_ptr; 

    //Data Elements
    struct DataNode *first_node_ptr;
    int data_node_count;
};

class QuadTree{

    ECS_Manager world;
    int max_depth;
    int max_node_capacity;
    Vector2D bott_left;
    Vector2D top_right;
    
    QuadNode *root_node_ptr;

    int find_quad_offset(Vector2D bott_left, Vector2D top_right, Vector2D pos);

    Vector2D get_pos_from_ID(int ID);

  public:
    
    QuadTree(ECS_Manager &world, Vector2D bott_left, Vector2D top_right, int max_depth, int max_node_capacity);
    void add_element(int ID, Vector2D pos);
    int  get_count();
       
};
