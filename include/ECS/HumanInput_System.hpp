#pragma once


#include "ECSManager.hpp"

#include "Input.hpp"
#include "ICommand.hpp"

struct human_input_config {
    
};

struct key_action_pair {
    InputKeyboardKey key_opt;
    ButtonAction button_action;

    ICommand *command;

};

struct button_action_pair {
    InputMouseButton key_opt;
    ButtonAction button_action;

    ICommand *command;
};


void HumanInput_System_Init(ECS_Manager &world, 
                            std::vector<struct key_action_pair> &key_action_pairs,
                            std::vector<struct button_action_pair> &button_action_pairs);

void HumanInput_add_key_action_pair(struct key_action_pair);

void HumanInput_System(ECS_Manager &world);
