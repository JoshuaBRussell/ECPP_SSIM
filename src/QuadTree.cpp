#include "QuadTree.hpp"
#include <cassert>
#include <cmath>
#include <stack>
#include <unordered_set>
#include <map> 

#include <raylib-cpp.hpp> // Temp for drawing QuadTree
#include "Render.hpp"


#include "ECSManager.hpp"
#include "Vector2D.hpp"

#include "./components/Position_comp.hpp"
#include "./components/Collision_comp.hpp"

static bool in_quadrant(Vector2D bott_left, Vector2D top_right, Vector2D pos){
        
    // Left/Bottom Boundary Inclusive
    bool in_x_bounds = (pos.x >= bott_left.x) && (pos.x < top_right.x);
    bool in_y_bounds = (pos.y >= bott_left.y) && (pos.y < top_right.y);
    
    return in_y_bounds && in_x_bounds;
}

QuadTree::QuadTree(ECS_Manager &world, Vector2D bott_left, Vector2D top_right, int max_depth, int max_node_capacity){
    
    this->world = world;
    this->bott_left = bott_left;
    this->top_right = top_right;
    
    this->max_depth = max_depth;
    this->max_node_capacity = max_node_capacity;
    
    this->root_node_ptr = new QuadNode;
    
    this->root_node_ptr->first_child_ptr = nullptr;
    this->root_node_ptr->first_node_ptr  = nullptr;
    this->root_node_ptr->data_node_count = 0;


    this->background_image = raylib::Image(640, 640, raylib::Color(0, 0, 0, 255));


}

raylib::Image *QuadTree::drawQuadTree(QuadNode* quad_node_ptr, Vector2D bott_left, int curr_depth){

    if (quad_node_ptr == nullptr){
        quad_node_ptr = this->root_node_ptr; 
        bott_left     = this->bott_left;
        curr_depth    = 0;
        this->background_image.Unload();
        this->background_image = raylib::Image(640, 640, raylib::Color(0, 0, 0, 255));
    }

    Vector2D root_w_h_vec = this->top_right - this->bott_left; 

    Vector2D w_h_vec = (1/std::pow(2.0, curr_depth)) * root_w_h_vec; 
     
    if(quad_node_ptr->first_child_ptr != nullptr){
        Vector2D curr_bl;  
        for (int child_offset = 0; child_offset < 4; child_offset++){
            curr_bl = bott_left;  
            // Find the bott_left for each tree
            if (child_offset == 0){
                curr_bl = curr_bl + 0.5*w_h_vec; 
            }
            if (child_offset == 1){
                curr_bl = curr_bl + Vector2D(0.0, w_h_vec.y/2);
            }
            if (child_offset == 2){
                curr_bl = curr_bl; //i.e. no change
            }
            if (child_offset == 3){
                curr_bl = curr_bl + Vector2D(w_h_vec.x/2, 0.0);
            }
            
            // Draw the children        
            this->drawQuadTree(quad_node_ptr->first_child_ptr + child_offset, curr_bl, curr_depth+1);
        }
 
    } 
    
    // Draw Current Node - if it doesn't have children
    // Removing the conditional causes the code to draw the QuadNodes
    // even if they are empty
    //if (quad_node_ptr->first_node_ptr != nullptr){
        int draw_offset = world2screenscale_Y(w_h_vec.y);
        raylib::Vector2 pos_vec(world2screen_X(bott_left.x), world2screen_Y(bott_left.y) - draw_offset);
        raylib::Vector2 size_vec(world2screenscale_X(w_h_vec.x), world2screenscale_Y(w_h_vec.y));
        //raylib::Vector2 size_vec(25, 25);
        //raylib::Vector2 pos_vec(320, 320);
        //raylib::Vector2 size_vec(25, 25); 
        raylib::Rectangle rec(pos_vec, size_vec);
        this->background_image.DrawRectangleLines(rec, 2, raylib::Color(255, 0, 0, 255));
        
        raylib::Vector2 count_offset_vec(world2screenscale_X(0.5*w_h_vec.x), world2screenscale_Y(0.5*w_h_vec.y));
        raylib::Vector2 obj_pos = pos_vec + count_offset_vec;
        this->background_image.DrawText(std::to_string(quad_node_ptr->data_node_count),
                                        obj_pos, 16.0, raylib::Color(255, 255, 255, 255));


    //}

    return &(this->background_image);
}

struct QuadNodeInfo {

    struct QuadNode *quad_node_ptr;
    Vector2D curr_bl;
    Vector2D curr_tr;

};

void QuadTree::add_element(int ID, Vector2D pos){
    
    // Add check to make sure this is even in the correct location
    assert(in_quadrant(this->bott_left, this->top_right, pos));

    // Get collision radius - assumes circular shaped object
    double radius = this->get_coll_radius_from_ID(ID);

    std::stack<struct QuadNodeInfo> quad_info_stack;

    struct QuadNodeInfo qn_info = {
        this->root_node_ptr,
        this->bott_left,
        this->top_right
    };
    quad_info_stack.push(qn_info);

    while(quad_info_stack.size() > 0){
        
        // Get top of stack
        // Pop it - to actually remove it
        struct QuadNodeInfo curr_node_info = quad_info_stack.top(); 
        quad_info_stack.pop();

        // Check if there needs to be a split here

        // It holds some portion of the element - else it wouldn't have been added
        // increment its count 
        curr_node_info.quad_node_ptr->data_node_count++;

        // If it's not a leaf, find out which of its children need be added to the stack 
        if(curr_node_info.quad_node_ptr->first_child_ptr != nullptr){ 
        
            // Go through any of its children. 
            // If any contain a region of the object,
            // add it to the stack to be processed 
            for (int child_offset = 0; child_offset < 4; child_offset++){
                
                struct QuadNode * curr_child_ptr = curr_node_info.quad_node_ptr->first_child_ptr + child_offset;

                Vector2D w_h_vec = curr_node_info.curr_tr - curr_node_info.curr_bl; 
                // Find child_bott_left
                Vector2D child_bott_left; 

                if (child_offset == 0){
                    child_bott_left = curr_node_info.curr_bl + 0.5*w_h_vec; 
                }
                if (child_offset == 1){
                    child_bott_left = curr_node_info.curr_bl + Vector2D(0.0, w_h_vec.y/2);
                }
                if (child_offset == 2){
                    child_bott_left = curr_node_info.curr_bl; //i.e. no change
                }
                if (child_offset == 3){
                    child_bott_left = curr_node_info.curr_bl + Vector2D(w_h_vec.x/2, 0.0);
                }

                Vector2D child_top_right = child_bott_left + 0.5*w_h_vec;

                // Add to the stack to be processed 
                if(is_in_region(pos, radius, child_bott_left, child_top_right)){
                    struct QuadNodeInfo qn_info = {
                        curr_child_ptr,
                        child_bott_left,
                        child_top_right
                    };
                    quad_info_stack.push(qn_info);                 
                } 
            } 


        
        } else {
             
            struct QuadNode *curr_quad_node = curr_node_info.quad_node_ptr;  
            // Add element
            struct DataNode *curr_data_node_ptr = new struct DataNode;
            curr_data_node_ptr->next = nullptr;
            curr_data_node_ptr->ID   = ID;
            
            // data_node_count for this node is incremented above
            this->add_element_to_node(curr_quad_node, curr_data_node_ptr);

            // Go ahead and insert it the map pair - even if it needs to split,
            // and be paired with the children
            this->leaf_map.insert({ID, curr_quad_node});
            
            // If needed, split the leaf into a parent
            if(curr_quad_node->data_node_count > this->max_node_capacity){
                this->split_and_distribute(curr_node_info);
            }
        } 

    }             
}

static void checkNodeCount(struct QuadNode* quad_node)
{
    if (quad_node == NULL)
        return;
 
    if (quad_node->first_child_ptr != nullptr){
        checkNodeCount(quad_node->first_child_ptr + 0);
        checkNodeCount(quad_node->first_child_ptr + 1);
        checkNodeCount(quad_node->first_child_ptr + 2);
        checkNodeCount(quad_node->first_child_ptr + 3); 
    
        // Check the parent adds to the children
        int children_sum = 0;
        for (int i = 0; i < 4; i++){
            struct QuadNode *curr_child_ptr = quad_node->first_child_ptr + i;
            children_sum += curr_child_ptr->data_node_count;
        }

        assert(quad_node->data_node_count == children_sum);

        std::cout << "P Sum: " << quad_node->data_node_count << std::endl;
        std::cout << "C Sum: " << children_sum << std::endl;
    }
}

std::unordered_set<int> QuadTree::remove_all_elements(){

    std::unordered_set<int> removed_ID_set;

    std::stack<struct QuadNodeInfo> quad_info_stack;

    struct QuadNodeInfo qn_info = {
        this->root_node_ptr,
        this->bott_left,
        this->top_right
    };
    
    quad_info_stack.push(qn_info);

    while(quad_info_stack.size() > 0){
        
        // Get top of stack
        // Pop it - to actually remove it
        struct QuadNodeInfo curr_node_info = quad_info_stack.top(); 
        quad_info_stack.pop(); 

        // If it's not a leaf
        if(curr_node_info.quad_node_ptr->first_child_ptr != nullptr){ 
            
            curr_node_info.quad_node_ptr->data_node_count = 0; // Go ahead and set its count back to zero
            
            // Go through its children. 
            // add it to the stack to be processed 
            for (int child_offset = 0; child_offset < 4; child_offset++){
                
                struct QuadNode * curr_child_ptr = curr_node_info.quad_node_ptr->first_child_ptr + child_offset;

                Vector2D w_h_vec = curr_node_info.curr_tr - curr_node_info.curr_bl; 
                // Find child_bott_left
                Vector2D child_bott_left; 

                if (child_offset == 0){
                    child_bott_left = curr_node_info.curr_bl + 0.5*w_h_vec; 
                }
                if (child_offset == 1){
                    child_bott_left = curr_node_info.curr_bl + Vector2D(0.0, w_h_vec.y/2);
                }
                if (child_offset == 2){
                    child_bott_left = curr_node_info.curr_bl; //i.e. no change
                }
                if (child_offset == 3){
                    child_bott_left = curr_node_info.curr_bl + Vector2D(w_h_vec.x/2, 0.0);
                }

                Vector2D child_top_right = child_bott_left + 0.5*w_h_vec;

                // Add to the stack to be processed 
                struct QuadNodeInfo qn_info = {
                    curr_child_ptr,
                    child_bott_left,
                    child_top_right
                };
                quad_info_stack.push(qn_info);                 
                 
            } 
        
        // It is a leaf, all data elements need to be removed
        } else {

            struct QuadNode *curr_quad_node = curr_node_info.quad_node_ptr;
            // Delete the parent's data nodes
            struct DataNode *curr_data_node_ptr = curr_quad_node->first_node_ptr; 
            while(curr_data_node_ptr != nullptr){
                
                removed_ID_set.insert(curr_data_node_ptr->ID);
                
                struct DataNode *temp_ptr = curr_data_node_ptr;
                curr_data_node_ptr = curr_data_node_ptr->next;
                delete temp_ptr;
                temp_ptr = nullptr;

                curr_quad_node->data_node_count--;
            }
            
            assert(curr_quad_node->data_node_count == 0);
            curr_quad_node->first_node_ptr = nullptr;
        }
    }

    return removed_ID_set;
}

void QuadTree::readd_all_elements(){
    std::unordered_set<int> element_ID_set = this->remove_all_elements();
    
    std::unordered_set<int>::iterator it;
    for (it = element_ID_set.begin(); it != element_ID_set.end(); ++it){
        this->add_element(*it, this->get_pos_from_ID(*it));
    }
}

void QuadTree::update(){

    this->leaf_map.clear();
    this->readd_all_elements();
    
    // Delete any nodes that are empty
    this->cleanup();

}

int  QuadTree::get_count(){
    return this->root_node_ptr->data_node_count;
}

// ---- Private ---- //
bool QuadTree::is_empty_leaf(struct QuadNode *node){
    return node->first_child_ptr == nullptr && node->first_node_ptr == nullptr;
}

void QuadTree::cleanup(){

    std::stack<struct QuadNode *> node_stack;
    
    // special check for root_node_ptr
    if (this->root_node_ptr->first_child_ptr != nullptr){
        node_stack.push(this->root_node_ptr); 
    }

    while (node_stack.size() > 0){
        
        struct QuadNode *curr_parent_node = node_stack.top(); 
        node_stack.pop();

        // Check to make sure each of the children are empty leaves
        int empty_leaf_count = 0;
        for (int i = 0; i < 4; i ++){
            struct QuadNode *curr_child_node = curr_parent_node->first_child_ptr + i; 
            
            if (is_empty_leaf(curr_child_node)){
                empty_leaf_count++; 
            }
            
            if (curr_child_node->first_child_ptr != nullptr){
                node_stack.push(curr_child_node); 
            } 
        }

        if (empty_leaf_count == 4){
            delete  curr_parent_node->first_child_ptr;
            curr_parent_node->first_child_ptr = nullptr;
        }
    }
}

std::unordered_set<int>* QuadTree::find_neighbors(int ID){
    
    std::unordered_set<int>* neigh_ID_set_ptr = new std::unordered_set<int>;
     
    std::multimap<int, struct QuadNode*>::iterator it;  
    for (it = this->leaf_map.lower_bound(ID); it != this->leaf_map.upper_bound(ID); ++it){
        
        struct QuadNode* curr_quad_node_ptr = it->second;
        struct DataNode* curr_data_node_ptr = curr_quad_node_ptr->first_node_ptr;
        
        while(curr_data_node_ptr != nullptr){
            neigh_ID_set_ptr->insert(curr_data_node_ptr->ID);
            curr_data_node_ptr = curr_data_node_ptr->next;
        }
    }
    
    return neigh_ID_set_ptr;
}

// Quadrant Tree 
//----|----//
//  1 |  0 //
//----|----//
//  2 |  3 //
//----|----//

// Could be moved to a static function and made to never be seen in the .hpp?
int QuadTree::find_quad_offset(Vector2D bott_left, Vector2D top_right, Vector2D pos){
       
    int return_value = -1;
    
    Vector2D w_h_vec = top_right - bott_left; // Width in X Coord. Height in Y Coord. 
 
    if (in_quadrant(bott_left + 0.5*w_h_vec, top_right, pos)){
        return 0;
    }
    if (in_quadrant(bott_left + Vector2D(0.0, w_h_vec.y/2), top_right - Vector2D(w_h_vec.x/2, 0.0), pos)){
        return 1;
    } 
    if (in_quadrant(bott_left, top_right - 0.5*w_h_vec, pos)){
        return 2;
    } 
    if (in_quadrant(bott_left + Vector2D(w_h_vec.x/2, 0.0), top_right - Vector2D(0.0, w_h_vec.y/2), pos)){
        return 3;
    } 

    return return_value;

}

// Assumes non-null pointers for the respective nodes.
// And that the data is already placed in the data_node
// The data_node_count data member must be incremented by the callee 
void QuadTree::add_element_to_node(struct QuadNode *quad_node_ptr, 
                                   struct DataNode *data_node_ptr){
    if(quad_node_ptr->first_node_ptr == nullptr){ 

        quad_node_ptr->first_node_ptr = data_node_ptr;
    
    } else {

        struct DataNode *temp_ptr = quad_node_ptr->first_node_ptr;

        quad_node_ptr->first_node_ptr = data_node_ptr;
        quad_node_ptr->first_node_ptr->next = temp_ptr;

    }
}

void QuadTree::split_and_distribute(struct QuadNodeInfo curr_node_info){
   
    struct QuadNode *node = curr_node_info.quad_node_ptr;
    Vector2D node_bl      = curr_node_info.curr_bl;
    Vector2D node_tr      = curr_node_info.curr_tr;
    Vector2D w_h_vec      = node_tr - node_bl;

    struct QuadNode *children = new QuadNode[4](); // The () ensure the children are default-init to 0.
    node->first_child_ptr = children;

    // Loop through each data_node  
    struct DataNode *curr_data_node_ptr = node->first_node_ptr;
    while(curr_data_node_ptr != nullptr){ 

        Vector2D pos    = this->get_pos_from_ID(curr_data_node_ptr->ID);
        double radius   = this->get_coll_radius_from_ID(curr_data_node_ptr->ID);

        // Remove the data node's association with the - now - parent quad node
        std::multimap<int, struct QuadNode*>::iterator it;  
        for (it = this->leaf_map.find(curr_data_node_ptr->ID); it != this->leaf_map.end(); ++it){
            if (it->second == node){
                this->leaf_map.erase(it); 
                break; // There should only be once instance of the {ID : parent_node_ptr} pair,
                       // so break once we find it. 
                       // Also, if there is only 1 instance in which ID is a key, the erasure might invalidate
                       // the iterator (it was causing an error, I I think this is why?)
            }
        } 


        // Since a data node element can inhabit multiple nodes, 
        // we loop through all children seeing if a data node 
        // inhabits any of them 
        for(int child_index = 0; child_index < 4; child_index++){
            
            Vector2D child_bl; 
            // Find the bott_left for each tree
            if (child_index == 0){
                child_bl = node_bl + 0.5*w_h_vec; 
            }
            if (child_index == 1){
                child_bl = node_bl + Vector2D(0.0, w_h_vec.y/2);
            }
            if (child_index == 2){
                child_bl = node_bl; //i.e. no change
            }
            if (child_index == 3){
                child_bl = node_bl + Vector2D(w_h_vec.x/2, 0.0);
            }

            Vector2D child_tr = child_bl + 0.5*w_h_vec; 
            
            if (this->is_in_region(pos, radius, child_bl, child_tr)){

                // Create a new data node element to then add to the child
                struct DataNode *child_data_node_ptr = new struct DataNode;
                child_data_node_ptr->ID   = curr_data_node_ptr->ID;
                child_data_node_ptr->next = nullptr; // Just to be safe


                (node->first_child_ptr + child_index)->data_node_count++;
                this->add_element_to_node((node->first_child_ptr + child_index), child_data_node_ptr);

                // Associate this element w/ the leaf
                this->leaf_map.insert({child_data_node_ptr->ID, (node->first_child_ptr + child_index)});
            } 

        }
        curr_data_node_ptr = curr_data_node_ptr->next; 
    }

    // Delete the parent's data nodes
    curr_data_node_ptr = node->first_node_ptr; 
    while(curr_data_node_ptr != nullptr){
        struct DataNode *temp_ptr = curr_data_node_ptr;
        curr_data_node_ptr = curr_data_node_ptr->next;
        delete temp_ptr;
        temp_ptr = nullptr;
    }
     
    node->first_node_ptr = nullptr;
}

Vector2D QuadTree::find_bott_left(int curr_depth, Vector2D bott_left, int child_offset){
    Vector2D root_w_h_vec = this->top_right - this->bott_left; 

    Vector2D w_h_vec = (1/std::pow(2.0, curr_depth)) * root_w_h_vec;

    Vector2D curr_bl = bott_left;  

    // Find the bott_left for each tree
    if (child_offset == 0){
        curr_bl = curr_bl + 0.5*w_h_vec; 
    }
    if (child_offset == 1){
        curr_bl = curr_bl + Vector2D(0.0, w_h_vec.y/2);
    }
    if (child_offset == 2){
        curr_bl = curr_bl; //i.e. no change
    }
    if (child_offset == 3){
        curr_bl = curr_bl + Vector2D(w_h_vec.x/2, 0.0);
    } 

    return curr_bl;
}

bool QuadTree::is_in_region(Vector2D pos, double radius, Vector2D bott_left, Vector2D top_right){

    return ((pos.x > (bott_left.x - radius)) && (pos.x < (top_right.x + radius))) && \
           ((pos.y > (bott_left.y - radius)) && (pos.y < (top_right.y + radius)));
}

Vector2D QuadTree::get_pos_from_ID(int ID){
    
    Position_Component *pos_comp_ptr = this->world.get_component<Position_Component>(ID);
    assert(pos_comp_ptr != nullptr);
    return pos_comp_ptr->position;     

}

double QuadTree::get_coll_radius_from_ID(int ID){

    Collision_Component *coll_comp_ptr = this->world.get_component<Collision_Component>(ID);
    assert(coll_comp_ptr != nullptr);
    return coll_comp_ptr->radius;

}


