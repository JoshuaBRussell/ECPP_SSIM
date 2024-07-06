#include "ConstraintVisual.hpp"
#include "ECSManager.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "../Examples/RigidBodyDoublePendulum/main.hpp"

#include "./components/Position_comp.hpp"
#include "./components/Constraint_comp.hpp"
#include "./components/Rotation_comp.hpp"

static size_t screen_width_in_pixels;
static size_t screen_height_in_pixels;
static double screen_width_in_meters;
static double screen_height_in_meters;


void Constraint_Visualization_Init(struct constr_visual_config &constr_visual_config){

    screen_width_in_pixels  = constr_visual_config.screen_width_in_pixels ;
    screen_height_in_pixels = constr_visual_config.screen_height_in_pixels;
    screen_width_in_meters  = constr_visual_config.screen_width_in_meters ;
    screen_height_in_meters = constr_visual_config.screen_height_in_meters;
}

void Constraint_Visualization_System(ECS_Manager &world){
    
    for (auto it = world.get_component_begin<Relative_Rot_Component>(); 
              it < world.get_component_end<Relative_Rot_Component>(); it++){
        
        // Take Physical coords. and convert to a location on the screen
        Eigen::Vector2d rb1_pos = world.get_component<Position_Component>(it->constr_entity1)->position;
        Eigen::Vector2d rb2_pos = world.get_component<Position_Component>(it->constr_entity2)->position;
    
        Eigen::Vector2d rb1_rel_pos = world.get_component<Relative_Rot_Component>(it->entity_id)->rel_body_pos1;
        Eigen::Vector2d rb2_rel_pos = world.get_component<Relative_Rot_Component>(it->entity_id)->rel_body_pos2;

        double rb1_rot = world.get_component<Rotation_Component>(it->constr_entity1)->angle;
        double rb2_rot = world.get_component<Rotation_Component>(it->constr_entity2)->angle;

        // Convert the constrained body point positions from body space to world space
        Eigen::Rotation2D<double> transform_matr1 = Eigen::Rotation2D<double>(rb1_rot);
        Eigen::Vector2d constr_pos1 = rb1_pos + transform_matr1 * rb1_rel_pos;
    
        Eigen::Rotation2D<double> transform_matr2 = Eigen::Rotation2D<double>(rb2_rot);
        Eigen::Vector2d constr_pos2 = rb2_pos + transform_matr2 * rb2_rel_pos;

        // Set the Constraint's Position as the Average of the intended constraint points of each body
        Eigen::Vector2d pos_delta = constr_pos2 - constr_pos1;
        
        Position_Component *constr_pos_comp_ptr = world.get_component<Position_Component>(it->entity_id);
        
        constr_pos_comp_ptr->position = constr_pos1 + 0.5*pos_delta;
    
    }
}


