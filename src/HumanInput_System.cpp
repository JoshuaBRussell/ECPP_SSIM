#include "HumanInput_System.hpp"


#include "ECSManager.hpp"



// The method of creating a (what is intended to be) a const vector, 
// with the initialization list apparently causes an excessive move/copy.
// It's not really an issue (premature optimization and all that), but
// I found this StackOverflow Questions Interesting:
// https://stackoverflow.com/questions/26457203/c-c11-efficient-way-to-have-static-array-vector-of-objects-initialized-with
static std::vector<struct key_action_pair> s_key_action_pairs; // "s_" so it would not cause a naming issue in
                                                               // _Init

void HumanInput_add_key_action_pair(struct key_action_pair key_action_pair){
    s_key_action_pairs.push_back(key_action_pair);
}

// This way of creating the vector on the stack then passing it into here to be copied(?)
// seems odd/wrong.
void HumanInput_System_Init(ECS_Manager &world, std::vector<struct key_action_pair> &key_action_pairs){
    s_key_action_pairs = key_action_pairs;
}

void HumanInput_System(ECS_Manager &world){
        
    for (auto it = s_key_action_pairs.begin(); it != s_key_action_pairs.end(); it++){

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
