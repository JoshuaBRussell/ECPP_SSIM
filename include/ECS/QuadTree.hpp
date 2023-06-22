#pragma once

#include <unordered_set>
#include <utility>

#include "ECSManager.hpp"
#include "Vector2D.hpp"

#include "Render.hpp"
#include "MemoryPool.hpp"

struct DataNode {
    int ID;

    int next;
};

struct QuadNode {
    
    //Children (if existent)
    struct QuadNode *first_child_ptr; 

    //Data Elements
    int first_node_index;
    int data_node_count;
 
};

class QuadTree{

    ECS_Manager world;
    int max_depth;
    int max_node_capacity;
    Vector2D bott_left;
    Vector2D top_right;

    std::unordered_set<int> id_set;

    std::multimap<int, struct QuadNode *> leaf_map;
    
    QuadNode *root_node_ptr; 

    int find_quad_offset(Vector2D bott_left, Vector2D top_right, Vector2D pos);
    Vector2D find_bott_left(int curr_depth, Vector2D bott_left, int child_offset);
    bool is_in_region(Vector2D pos, double radius, Vector2D child_bott_left, Vector2D child_top_right);   
    
    void remove_element_from_leaf_map(int ID, struct QuadNode *quad_node_ptr);
    void remove_all_elements_from_leaves();
    void readd_all_elements();
    bool is_empty_leaf(struct QuadNode *node); 
    void cleanup();
    
    void add_element_to_node(struct QuadNode *quad_node_ptr, int index);
    void split_and_distribute(struct QuadNodeInfo curr_node_info);

    Vector2D get_pos_from_ID(int ID);
    double get_coll_radius_from_ID(int ID);

  public:
    
    MemoryPool<struct DataNode> data_nodes;

    QuadTree(ECS_Manager &world, Vector2D bott_left, Vector2D top_right, int max_depth, int max_node_capacity);
    void add_element(int ID, Vector2D pos);
    void update(); 
    int  get_count();

    // Mem must be deleted for this pointer
    std::unordered_set<int>* find_neighbors(int ID);


    raylib::Image *drawQuadTree(QuadNode* quad_node_ptr, Vector2D bott_left, int curr_depth);
    raylib::Image background_image; 
    

};
