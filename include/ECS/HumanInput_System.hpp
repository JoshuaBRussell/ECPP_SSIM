#pragma once


#include "ECSManager.hpp"

#include "Input.hpp"
#include "ICommand.hpp"

struct human_input_config {
    
};

struct key_action_pair {
    InputKeyboardKey key_opt;
    KeyboardKeyAction key_action;

    ICommand *command;

};


void HumanInput_System_Init(ECS_Manager &world, struct human_input_config &human_input_config);

void HumanInput_add_key_action_pair(struct key_action_pair);

void HumanInput_System(ECS_Manager &world);
