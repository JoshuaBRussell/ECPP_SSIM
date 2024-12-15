#include "HumanInput_System.hpp"


#include "ECSManager.hpp"


static std::vector<struct key_action_pair> *key_action_pairs_ptr;
static std::vector<struct button_action_pair> *button_action_pairs_ptr;


void HumanInput_add_key_action_pair(struct key_action_pair key_action_pair){
    key_action_pairs_ptr->push_back(key_action_pair);
}

void HumanInput_add_button_action_pair(struct button_action_pair button_action_pair){
    button_action_pairs_ptr->push_back(button_action_pair);
}

void HumanInput_System_Init(ECS_Manager &world,
                            std::vector<struct key_action_pair> &key_action_pairs,
                            std::vector<struct button_action_pair> &button_action_pairs){

    key_action_pairs_ptr = new std::vector<struct key_action_pair>(key_action_pairs);
    button_action_pairs_ptr = new std::vector<struct button_action_pair>(button_action_pairs);
}

void HumanInput_System(ECS_Manager &world){

    // Poll Keyboard Keys
    if (key_action_pairs_ptr){ // Possible + Valid the user didn't pass in a key-action pair vector
        for (auto it = key_action_pairs_ptr->begin(); it != key_action_pairs_ptr->end(); it++){

            bool should_act = false;

            // Only have to support small and finite number of key/button actions 
            switch(it->button_action) {

                case PRESSED:
                    should_act = Input_is_key_pressed(it->key_opt);
                    break;

                case PRESSED_REPEAT:
                    should_act = Input_is_key_pressed_repeat(it->key_opt);
                    break;

                case DOWN:
                    should_act = Input_is_key_down(it->key_opt);
                    break;

                case RELEASED:
                    should_act = Input_is_key_released(it->key_opt);
                    break;

                case UP:
                    should_act = Input_is_key_up(it->key_opt);
                    break;

            }

            if (should_act){
                it->command->execute(world);
            }
        }
    }

    // Poll Mouse Buttons
    if (button_action_pairs_ptr){ // Possible + Valid the user didn't pass in a button-action pair vector
        for (auto it = button_action_pairs_ptr->begin(); it != button_action_pairs_ptr->end(); it++){

            bool should_act = false;

            // Only have to support small and finite number of key/button actions 
            switch(it->button_action) {

                case PRESSED:
                    should_act = Input_is_mouse_button_pressed(it->key_opt);
                    break;

                case PRESSED_REPEAT:
                    // N/A - currently not supported
                    break;

                case DOWN:
                    should_act = Input_is_mouse_button_down(it->key_opt);
                    break;

                case RELEASED:
                    should_act = Input_is_mouse_button_released(it->key_opt);
                    break;

                case UP:
                    should_act = Input_is_mouse_button_up(it->key_opt);
                    break;

            }

            if (should_act){
                it->command->execute(world);
            }
        }
    }

}
