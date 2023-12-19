#pragma once

#include <unordered_set>
#include <utility>

#include <Eigen/Core>

#include "ECSManager.hpp"

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
    Eigen::Vector2f bott_left;
    Eigen::Vector2f top_right;

    std::unordered_set<int> id_set;

    std::multimap<int, struct QuadNode *> leaf_map;
    
    QuadNode *root_node_ptr; 

    int find_quad_offset(Eigen::Vector2f bott_left, Eigen::Vector2f top_right, Eigen::Vector2f pos);
    Eigen::Vector2f find_bott_left(int curr_depth, Eigen::Vector2f bott_left, int child_offset);
    bool is_in_region(Eigen::Vector2f pos, double radius, Eigen::Vector2f child_bott_left, Eigen::Vector2f child_top_right);   
    
    void remove_element_from_leaf_map(int ID, struct QuadNode *quad_node_ptr);
    void remove_all_elements_from_leaves();
    void readd_all_elements();
    bool is_empty_leaf(struct QuadNode *node); 
    void cleanup();
    
    void add_element_to_node(struct QuadNode *quad_node_ptr, int index);
    void split_and_distribute(struct QuadNodeInfo curr_node_info);

    Eigen::Vector2f get_pos_from_ID(int ID);
    double get_coll_radius_from_ID(int ID);

  public:
    
    MemoryPool<struct DataNode> data_nodes;

    QuadTree(ECS_Manager &world, Eigen::Vector2f bott_left, Eigen::Vector2f top_right, int max_depth, int max_node_capacity);
    void add_element(int ID, Eigen::Vector2f pos);
    void update(); 
    int  get_count();

    // Mem must be deleted for this pointer
    std::unordered_set<int>* find_neighbors(int ID);


    raylib::Image *drawQuadTree(QuadNode* quad_node_ptr, Eigen::Vector2f bott_left, int curr_depth);
    raylib::Image background_image; 
    

};
