#pragma once

#include "ECS.hpp"
#include "ECSManager.hpp"

#include <Eigen/Core>

// ---- Graphics System ---- //
// Pre-Graphics System
// - Intepretation Systems (e.g. Constraint Visualization)
// 
// Render(ers)
// - Texture Renderer / Draw Calls
//
// Dear ImGui GUI/Plots

struct render_config {
    
    int screen_width_in_pixels;
    int screen_height_in_pixels;
    std::string window_title;
    
    int target_fps;
};

void Render_System_Init(ECS_Manager &world, struct render_config &render_config);
void Render_System_Shutdown(); 

bool Render_System_WindowShouldClose();

// Automatically have Systems called before/after the world rendering
void Render_System_add_pre_render(void (*sys)(ECS_Manager&));
void Render_System_add_post_render(void (*sys)(ECS_Manager&));

void Render_System_Exclusive(ECS_Manager &world);
void Render_System_NonExclusive(ECS_Manager &world);
void Render_System(ECS_Manager &world);

// Let the Render System handle input events since it is easier

typedef enum {
    NULL_KEY            = 0,    // NULL, used for no key pressed
    // Alphanumeric keys
    APOSTROPHE_KEY      = 39,   // '
    COMMA_KEY           = 44,   // ,
    MINUS_KEY           = 45,   // -
    PERIOD_KEY          = 46,   // .
    SLASH_KEY           = 47,   // /
    ZERO_KEY            = 48,   // 0
    ONE_KEY             = 49,   // 1
    TWO_KEY             = 50,   // 2
    THREE_KEY           = 51,   // 3
    FOUR_KEY            = 52,   // 4
    FIVE_KEY            = 53,   // 5
    SIX_KEY             = 54,   // 6
    SEVEN_KEY           = 55,   // 7
    EIGHT_KEY           = 56,   // 8
    NINE_KEY            = 57,   // 9
    SEMICOLON_KEY       = 59,   // ;
    EQUAL_KEY           = 61,   // =
    A_KEY               = 65,   // A | a
    B_KEY               = 66,   // B | b
    C_KEY               = 67,   // C | c
    D_KEY               = 68,   // D | d
    E_KEY               = 69,   // E | e
    F_KEY               = 70,   // F | f
    G_KEY               = 71,   // G | g
    H_KEY               = 72,   // H | h
    I_KEY               = 73,   // I | i
    J_KEY               = 74,   // J | j
    K_KEY               = 75,   // K | k
    L_KEY               = 76,   // L | l
    M_KEY               = 77,   // M | m
    N_KEY               = 78,   // N | n
    O_KEY               = 79,   // O | o
    P_KEY               = 80,   // P | p
    Q_KEY               = 81,   // Q | q
    R_KEY               = 82,   // R | r
    S_KEY               = 83,   // S | s
    T_KEY               = 84,   // T | t
    U_KEY               = 85,   // U | u
    V_KEY               = 86,   // V | v
    W_KEY               = 87,   // W | w
    X_KEY               = 88,   // X | x
    Y_KEY               = 89,   // Y | y
    Z_KEY               = 90,   // Z | z
    LEFT_BRACKET_KEY    = 91,   // [
    BACKSLASH_KEY       = 92,   // '\'
    RIGHT_BRACKET_KEY   = 93,   // ]
    GRAVE_KEY           = 96,   // `
    // Function keys
    SPACE_KEY           = 32,   // Space
    ESCAPE_KEY          = 256,  // Esc
    ENTER_KEY           = 257,  // Enter
    TAB_KEY             = 258,  // Tab
    BACKSPACE_KEY       = 259,  // Backspace
    INSERT_KEY          = 260,  // Ins
    DELETE_KEY          = 261,  // Del
    RIGHT_KEY           = 262,  // Cursor right
    LEFT_KEY            = 263,  // Cursor left
    DOWN_KEY            = 264,  // Cursor down
    UP_KEY              = 265,  // Cursor up
    PAGE_UP_KEY         = 266,  // Page up
    PAGE_DOWN_KEY       = 267,  // Page down
    HOME_KEY            = 268,  // Home
    END_KEY             = 269,  // End
    CAPS_LOCK_KEY       = 280,  // Caps lock
    SCROLL_LOCK_KEY     = 281,  // Scroll down
    NUM_LOCK_KEY        = 282,  // Num lock
    PRINT_SCREEN_KEY    = 283,  // Print screen
    PAUSE_KEY           = 284,  // Pause
    F1_KEY              = 290,  // F1
    F2_KEY              = 291,  // F2
    F3_KEY              = 292,  // F3
    F4_KEY              = 293,  // F4
    F5_KEY              = 294,  // F5
    F6_KEY              = 295,  // F6
    F7_KEY              = 296,  // F7
    F8_KEY              = 297,  // F8
    F9_KEY              = 298,  // F9
    F10_KEY             = 299,  // F10
    F11_KEY             = 300,  // F11
    F12_KEY             = 301,  // F12
    LEFT_SHIFT_KEY      = 340,  // Shift left
    LEFT_CONTROL_KEY    = 341,  // Control left
    LEFT_ALT_KEY        = 342,  // Alt left
    LEFT_SUPER_KEY      = 343,  // Super left
    RIGHT_SHIFT_KEY     = 344,  // Shift right
    RIGHT_CONTROL_KEY   = 345,  // Control right
    RIGHT_ALT_KEY       = 346,  // Alt right
    RIGHT_SUPER_KEY     = 347,  // Super right
    KB_MENU_KEY         = 348,  // KB menu
} RenderKeyboardKey;

bool Render_is_key_pressed(int key);
bool Render_is_key_pressed_repeat(int key);
bool Render_is_key_down(int key);
bool Render_is_key_released(int key);
bool Render_is_key_up(int key);
int Render_get_key_pressed(void);                                // Get key pressed (keycode), call it multiple times for keys queued
                                                                 // , returns 0 when the queue is empty
int Render_get_char_pressed(void);                               // Get char pressed (unicode), call it multiple times for chars 
                                                                 // queued, returns 0 when the queue is empty

typedef enum {
    LEFT_MOUSE_BUTTON    = 0,
    RIGHT_MOUSE_BUTTON   = 1,
    MIDDLE_MOUSE_BUTTON  = 2,
    SIDE_MOUSE_BUTTON    = 3,
    EXTRA_MOUSE_BUTTON   = 4,
    FORWARD_MOUSE_BUTTON = 5,
    BACK_MOUSE_BUTTON    = 6,
} RenderMouseButton;

bool Render_is_mouse_button_pressed(int button);
bool Render_is_mouse_button_down(int button);
bool Render_is_mouse_button_released(int button);
bool Render_is_mouse_button_up(int button);
int Render_get_mouse_X(void);
int Render_get_mouse_Y(void);
Eigen::Vector2d Render_get_mouse_position();

// ---- Util Functions ---- //

// The idea was that the scale functions would just transform the (...)scale_X/Y functions
// would handle the scale factor - esque conversions
//
// The (...)_X/Y functions would handle the transforms. 

// I don't like this and it either needs to change or be made more clear which is which.
double screen2worldscale_X(int screen_x, double screen_width_in_meters);
double screen2worldscale_Y(int screen_y, double screen_height_in_meters);

double screen2world_Y(int screen_y, double screen_height_in_meters);
double screen2world_X(int screen_x, double screen_width_in_meters);

//Scale differences
double world2screenscale_X(double x, double screen_width_in_meters);
double world2screenscale_Y(double y, double screen_height_in_meters);

// Coord transform that assumes orthogonality for the transform
double world2screen_X(double x, double screen_width_in_meters);
double world2screen_Y(double y, double screen_height_in_meters);
