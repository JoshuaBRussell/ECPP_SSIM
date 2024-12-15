#pragma once

#include "ECSManager.hpp"

// ---- Abstract Command ---- //
class ICommand {
  
  public:
    virtual void execute(ECS_Manager &world) = 0;

};







