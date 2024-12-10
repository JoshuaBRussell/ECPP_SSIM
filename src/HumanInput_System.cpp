#include "HumanInput_System.hpp"


#include "ECSManager.hpp"
#include "./../Examples/HumanInputController/Commands.hpp"


static std::vector<struct key_action_pair> key_action_pairs;

void HumanInput_add_key_action_pair(struct key_action_pair key_action_pair){
    key_action_pairs.push_back(key_action_pair);
}


void HumanInput_System_Init(ECS_Manager &world, struct human_input_config &human_input_config){

}

void HumanInput_System(ECS_Manager &world){
        
    for (auto it = key_action_pairs.begin(); it != key_action_pairs.end(); it++){

        bool should_act = false;
        
        // Only have to support small and finite number of key/button actions
        if (it->key_action == PRESSED){
            should_act = Input_is_key_pressed(it->key_opt);
        }

        if (should_act){
            it->command->execute(world);
        }
    }

}
