#pragma once

#include "ECS.hpp"
#include "ECSManager.hpp"

#include <raylib-cpp.hpp>

// The idea was that the scale functions would just transform the (...)scale_X/Y functions
// would handle the scale factor - esque conversions
//
// The (...)_X/Y functions would handle the transforms. 
//
// I don't like this and it either needs to change or be made more clear which is which.
static double screen2worldscale_X(int screen_x){
    return screen_x * ((double)SCREEN_WIDTH_METERS/SCREEN_WIDTH_IN_PIXELS); 
};

static double screen2worldscale_Y(int screen_y){
    return -(screen_y - SCREEN_HEIGHT_IN_PIXELS)*(SCREEN_HEIGHT_METERS/SCREEN_HEIGHT_IN_PIXELS);
};

static double screen2world_Y(int screen_y){
    return  -(SCREEN_HEIGHT_METERS/(double)SCREEN_HEIGHT_IN_PIXELS) * (double)(screen_y - SCREEN_HEIGHT_IN_PIXELS);
};

//Scale differences
static double world2screenscale_X(double x){
    return x * ((double)SCREEN_WIDTH_IN_PIXELS/SCREEN_WIDTH_METERS);  
}
static double world2screenscale_Y(double y){
    return y * ((double)SCREEN_HEIGHT_IN_PIXELS/SCREEN_HEIGHT_METERS);
}

// Coord transform that assumes orthogonality for the transform
static double world2screen_X(double x){
    return world2screenscale_X(x);
}
static double world2screen_Y(double y){
    return -world2screenscale_Y(y) + (double)SCREEN_HEIGHT_IN_PIXELS;
}

void Render_init();

void Render_System_Exclusive(ECS_Manager &world);
void Render_System_NonExclusive(ECS_Manager &world);
