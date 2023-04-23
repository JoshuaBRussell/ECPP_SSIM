#include "QuadTree.hpp"

#include <cassert>

#include "Vector2D.hpp"

static bool in_quadrant(Vector2D bott_left, Vector2D top_right, Vector2D pos){
        
    // Left/Bottom Boundary Inclusive
    bool in_x_bounds = (pos.x >= bott_left.x) && (pos.x < top_right.x);
    bool in_y_bounds = (pos.y >= bott_left.y) && (pos.y < top_right.y);
    
    return in_y_bounds && in_x_bounds;
}


QuadTree::QuadTree(Vector2D bott_left, Vector2D top_right, int max_depth){
    
    this->bott_left = bott_left;
    this->top_right = top_right;
    
    this->root_node_ptr = new QuadNode;
    
    this->root_node_ptr->first_child_ptr = nullptr;
    this->root_node_ptr->first_node_ptr  = nullptr;
    this->root_node_ptr->data_node_count = 0;
}

void QuadTree::add_element(int ID, Vector2D pos){
    
    // Add check to make sure this is even in the correct location
    assert(in_quadrant(this->bott_left, this->top_right, pos));

    int curr_depth = 0; 
    Vector2D curr_bl = this->bott_left;
    Vector2D curr_tr = this->top_right;
    QuadNode *curr_quad_node_ptr = this->root_node_ptr;
    
    // If the child has a non-null first_child_ptr, it must have children.
    // Find out which child to place into
    while(curr_depth <= this->max_depth && curr_quad_node_ptr->first_child_ptr != nullptr){
        
        // This could also be found using the curr_depth variable: (1/2^N) * (root_width, root_height)
        // Where N is the curent depth.
        // This would be useful if there is a way to get rid of using the curr_br/curr_tl vars
        Vector2D w_h_vec = curr_tr - curr_bl; // Width in X Coord. Height in Y Coord. 
        // Find out what child to traverse next 
        int child_offset = find_quad_offset(curr_bl, curr_tr, pos); 
        
        // Find new bott_left
        if (0){
            curr_bl = curr_bl + 0.5*w_h_vec; 
        }
        if (1){
            curr_bl = curr_bl + Vector2D(0.0, w_h_vec.y/2);
        }
        if (2){
            curr_bl = curr_bl; //i.e. no change
        }
        if (3){
            curr_bl = curr_bl + Vector2D(w_h_vec.x/2, 0.0);
        } 
        
        // Find new top_right
        curr_tr = curr_bl + 0.5 * w_h_vec;

        curr_quad_node_ptr = curr_quad_node_ptr->first_child_ptr + child_offset;    
        curr_depth++;
    }
    
    struct DataNode *curr_data_node_ptr = curr_quad_node_ptr->first_node_ptr;
    while(curr_data_node_ptr != nullptr){
        // Increment the parent's count since its children will get it too.
        // This currently doesn't assume there whill be any type of non-crashable behavior.
        // So either a child succesfully adds the new element - or it doesn't and the program
        // crashes, so there is no need to account for this. 
        curr_quad_node_ptr->data_node_count++; 
        curr_data_node_ptr = curr_data_node_ptr->next; 
    }
 
    curr_data_node_ptr = new struct DataNode;
    curr_quad_node_ptr->data_node_count++; 
    curr_data_node_ptr->next = nullptr;
    curr_data_node_ptr->ID   = ID; 

    //Decide if it needs to split - if so, distribute elements amongst the children 
     
}

int  QuadTree::get_count(){
    return this->root_node_ptr->data_node_count;
}

// ---- Private ---- //

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
