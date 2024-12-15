#pragma once

#include "main.hpp"

#include "ECSManager.hpp"
#include "ICommand.hpp"

#include "Input.hpp"
#include "RigidBodyUtil.hpp"

#include "Render.hpp"
#include "Constraint.hpp"

#include "./ECS/components/Position_comp.hpp"


// ---- Commands ----//

class AddNewComplexObj : public ICommand {
  public:
    void execute(ECS_Manager &world) override {
       add_complex_shape_to_world(world);
       Constraint_System_ReInit(world); 
    }
};

class DeleteEntNearMouse : public ICommand {
  public:
    void execute (ECS_Manager &world) override {
        // Convert Mouse Position to World Space
        Eigen::Vector2d mouse_pos = Input_get_mouse_position();
        Eigen::Vector2d mouse_pos_world = Eigen::Vector2d(screen2world_X(mouse_pos(0), SCREEN_WIDTH_METERS), 
                                                          screen2world_Y(mouse_pos(1), SCREEN_HEIGHT_METERS));
        // Find the closest entity - if there is even one within some range
        // Find all entity's positions
        double min_dist = 1000.0;
        int min_dist_entity = -1;
        for (auto it = world.get_component_begin<Position_Component>(); 
              it < world.get_component_end<Position_Component>(); it++){
            
            double squared_dist = pow(mouse_pos_world(0) - it->position(0),2) + pow(mouse_pos_world(1) - it->position(1), 2);
            
            if (squared_dist < min_dist) {
                min_dist = squared_dist;
                min_dist_entity = it->entity_id;
            }

        }

        world.destroy_entity(min_dist_entity);
    }
};
