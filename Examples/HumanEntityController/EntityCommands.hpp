#pragma once

#include "ECSManager.hpp"
#include "ICommand.hpp"

#include "EntityControl_comp.hpp"


// ---- Commands ----//

class SendUpCommand : public ICommand {
  public:
    void execute (ECS_Manager &world) override {
        for (auto it = world.get_component_begin<EntityControl_Component>(); 
              it < world.get_component_end<EntityControl_Component>(); it++){
            
            it->comm_dir = CommandDirections::UP_COMM;              
        }
    }
};

class SendDownCommand : public ICommand {
  public:
    void execute (ECS_Manager &world) override {
        for (auto it = world.get_component_begin<EntityControl_Component>(); 
              it < world.get_component_end<EntityControl_Component>(); it++){
            
            it->comm_dir = CommandDirections::DOWN_COMM;              
        }
    }
};

class SendLeftCommand : public ICommand {
  public:
    void execute (ECS_Manager &world) override {
        for (auto it = world.get_component_begin<EntityControl_Component>(); 
              it < world.get_component_end<EntityControl_Component>(); it++){
            
            it->comm_dir = CommandDirections::LEFT_COMM;              
        }
    }
};

class SendRightCommand : public ICommand {
  public:
    void execute (ECS_Manager &world) override {
        for (auto it = world.get_component_begin<EntityControl_Component>(); 
              it < world.get_component_end<EntityControl_Component>(); it++){
            
            it->comm_dir = CommandDirections::RIGHT_COMM;              
        }
    }
};
