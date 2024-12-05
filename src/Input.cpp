
#include "Input.hpp"

#include "raylib.h"

// Most of these will be simple wrapper around the current libs functions until (if ever) a different
// input lib is used

// Keyboard Input //
bool Input_is_key_pressed(int key){
    return IsKeyPressed(key);
}

bool Input_is_key_pressed_repeat(int key){
    return IsKeyPressedRepeat(key);
}

bool Input_is_key_down(int key){
    return IsKeyDown(key);
}

bool Input_is_key_released(int key){
    return IsKeyReleased(key);
}

bool Input_is_key_up(int key){
    return IsKeyUp(key);
}

int Input_get_key_pressed(void){
    return GetKeyPressed();
}

int Input_get_char_pressed(void){
    return GetCharPressed();
}

// Mouse Input
bool Input_is_mouse_button_pressed(int button){
    return IsMouseButtonPressed(button);
}

bool Input_is_mouse_button_down(int button){
    return IsMouseButtonDown(button);
}

bool Input_is_mouse_button_released(int button){
    return IsMouseButtonReleased(button);
}

bool Input_is_mouse_button_up(int button){
    return IsMouseButtonUp(button);
}

int Input_get_mouse_X(void){
    return GetMouseX();
}

int Input_get_mouse_Y(void){
    return GetMouseY();
}

Eigen::Vector2d Input_get_mouse_position(){
    Vector2 val = GetMousePosition(); // Use a intermediary value to avoid a depence on the lower level lib
    // Just return a Eigen vector instead
    return Eigen::Vector2d(val.x, val.y);
}
