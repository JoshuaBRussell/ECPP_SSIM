#include "HumanInput_System.hpp"


#include "ECSManager.hpp"
#include "./../Examples/HumanInputController/Commands.hpp"

void HumanInput_System_Init(ECS_Manager &world, struct human_input_config &human_input_config){

}

void HumanInput_System(ECS_Manager &world){

        if (Input_is_key_pressed(R_KEY)){
            AddNewComplexObj comm;
            comm.execute(world);
        }

        if (Input_is_mouse_button_pressed(LEFT_MOUSE_BUTTON)){
            DeleteEntNearMouse comm;
            comm.execute(world);
        }

}
