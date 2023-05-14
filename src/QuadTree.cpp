#include "QuadTree.hpp"
#include <cassert>
#include <cmath>
#include <stack>

#include <raylib-cpp.hpp> // Temp for drawing QuadTree
#include "Render.hpp"


#include "ECSManager.hpp"
#include "Vector2D.hpp"

#include "./components/Position_comp.hpp"

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
    //}

    return &(this->background_image);
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
        
        // Find new top_right
        curr_tr = curr_bl + 0.5 * w_h_vec;

        // Add check to make sure this is even in the correct location
        assert(in_quadrant(curr_bl, curr_tr, pos));

        // Increment the parent's count since its children will get it too.
        // This currently doesn't assume there whill be any type of non-crashable behavior.
        // So either a child succesfully adds the new element - or it doesn't and the program
        // crashes, so there is no need to account for this. 
        curr_quad_node_ptr->data_node_count++; 

        curr_quad_node_ptr = curr_quad_node_ptr->first_child_ptr + child_offset;    
        curr_depth++; 

        std::cout << curr_depth << std::endl;
    }
     
    struct QuadNode *parent = curr_quad_node_ptr; 
    // Decide if it needs to split - if so, distribute current elements amongst the children
    // data_node_count was incremented above 
    if (curr_quad_node_ptr->data_node_count >= this->max_node_capacity){
        
        struct QuadNode *children = new QuadNode[4](); // The () ensure the children are default-init to 0.
        curr_quad_node_ptr->first_child_ptr = children;
         
        for (int i = 0; i < curr_quad_node_ptr->data_node_count; i++){

            if(curr_quad_node_ptr->first_node_ptr == nullptr){
                std::cout << "Debug Break Point" << std::endl;
            }
           
            Vector2D elem_pos = get_pos_from_ID(curr_quad_node_ptr->first_node_ptr->ID);

            int child_offset = this->find_quad_offset(curr_bl, curr_tr, elem_pos);
            struct QuadNode *curr_child_node = children + child_offset;

            if (curr_child_node->first_node_ptr == nullptr){
                curr_child_node->first_node_ptr = curr_quad_node_ptr->first_node_ptr;
                
                // After the node at the top of the curr_quad node is pointed to by a child,
                // have the parent node point to its next child.
                // This should never cause an issue since this is done at a splitting point. 
                curr_quad_node_ptr->first_node_ptr = curr_quad_node_ptr->first_node_ptr->next; 
               
                curr_child_node->first_node_ptr->next = nullptr;
            
            } else {
                struct DataNode *temp = curr_child_node->first_node_ptr;
                curr_child_node->first_node_ptr = curr_quad_node_ptr->first_node_ptr;
                
                // After the node at the top of the curr_quad node is pointed to by a child,
                // have the parent node point to its next child.
                // This should never cause an issue since this is done at a splitting point. 
                curr_quad_node_ptr->first_node_ptr = curr_quad_node_ptr->first_node_ptr->next; 
                
                curr_child_node->first_node_ptr->next = temp;
            }
            
            curr_child_node->data_node_count++;
        }

        // Update curr_quad_node_ptr to point to the newest entry's appropriate QuadNode
        int child_offset = this->find_quad_offset(curr_bl, curr_tr, pos);       
        curr_quad_node_ptr = children + child_offset;

    }

    // Add the newest entry
    if (curr_quad_node_ptr->first_node_ptr == nullptr){
        struct DataNode *curr_data_node_ptr = new struct DataNode;
        curr_data_node_ptr->next = nullptr;
        curr_data_node_ptr->ID   = ID; 

        curr_quad_node_ptr->first_node_ptr = curr_data_node_ptr; 
         
    } else {
        struct DataNode *temp = curr_quad_node_ptr->first_node_ptr;
     
        curr_quad_node_ptr->first_node_ptr = new struct DataNode; 
        curr_quad_node_ptr->first_node_ptr->next = temp;
        curr_quad_node_ptr->first_node_ptr->ID   = ID;  
    }
    
    curr_quad_node_ptr->data_node_count++;  
    if (parent->first_child_ptr != nullptr){
        for (int i = 0; i < 4; i++){
            std::cout << "HERE: " << static_cast<int>((parent->first_child_ptr+i)->data_node_count) << std::endl;
        }
    }
      
     
}

std::vector<DataNode *> QuadTree::find_and_remove_invalids(QuadNode* quad_node_ptr, Vector2D bott_left, int curr_depth){
   
    std::vector<DataNode *> invalids;

    if(quad_node_ptr->first_child_ptr != nullptr){ 
        
        for (int child_offset = 0; child_offset < 4; child_offset++){
            Vector2D curr_bl = this->find_bott_left(curr_depth, bott_left, child_offset); 
            std::vector<DataNode *> temp_invalids = this->find_and_remove_invalids(quad_node_ptr->first_child_ptr + child_offset, curr_bl, curr_depth+1);
            invalids.insert(std::end(invalids), std::begin(temp_invalids), std::end(temp_invalids));
        }
    
    } else {

        Vector2D root_w_h_vec = this->top_right - this->bott_left; 

        Vector2D w_h_vec = (1/std::pow(2.0, curr_depth)) * root_w_h_vec; 
        
        // Find Invalid Elements
        Vector2D top_right = bott_left + w_h_vec; 
         
        if (quad_node_ptr->first_node_ptr != nullptr){
            
            // Handle removing from Front
            Vector2D pos = this->get_pos_from_ID(quad_node_ptr->first_node_ptr->ID);    
            while(!in_quadrant(bott_left, top_right, pos) && quad_node_ptr->first_node_ptr != nullptr){
                std::cout << "Found Invalids" << std::endl; 
                DataNode *temp = quad_node_ptr->first_node_ptr->next;
                invalids.push_back(quad_node_ptr->first_node_ptr);
                quad_node_ptr->first_node_ptr = temp;

                quad_node_ptr->data_node_count--;
        
                if(quad_node_ptr->first_node_ptr){
                    pos = this->get_pos_from_ID(quad_node_ptr->first_node_ptr->ID); 
                } 
            } 
            
            // Handle removing any past the first
            if (quad_node_ptr->first_node_ptr != nullptr){
                // First data node already handled w/ above loop
                DataNode *prev_data_node_ptr = quad_node_ptr->first_node_ptr; 
                while(prev_data_node_ptr->next != nullptr){
                    Vector2D pos = this->get_pos_from_ID(prev_data_node_ptr->next->ID); 
                    if(!in_quadrant(bott_left, top_right, pos)){
                        std::cout << "Found Invalids" << std::endl;
                        DataNode *temp = prev_data_node_ptr->next->next;
                        invalids.push_back(prev_data_node_ptr->next);
                        quad_node_ptr->data_node_count--;         
                        // Since the curr node under scrutiny was invalid, the prev_data_node_ptr stays the same
                        prev_data_node_ptr->next = temp; 
                    } else{
                        prev_data_node_ptr = prev_data_node_ptr->next;
                    }
                }
            }

        }  

    }
    
    // Handle the rest 
    
    return invalids;
}

void QuadTree::update(){
    
    std::vector<DataNode *> invalids = this->find_and_remove_invalids(this->root_node_ptr, this->bott_left, 0);
    
    // Place the invalids back in the tree
    int id;
    Vector2D pos;
    for (int i = 0; i < invalids.size(); i++){
        
        // Add the same data back in - this should be changed to NOT allocate/free memory
        id  = invalids[i]->ID;
        pos = this->get_pos_from_ID(id);
        this->add_element(id, pos);
        
        // Free Node
        delete invalids[i];   
        
    }

    // Delete any nodes that are still empty
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

Vector2D QuadTree::get_pos_from_ID(int ID){
   return this->world.get_component<Position_Component>(ID)->position;     
}


