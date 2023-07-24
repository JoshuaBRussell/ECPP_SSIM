#pragma once

#include "ECSManager.hpp"
#include "QuadTree.hpp"

#define VISUAL_RADIUS 0.5
#define PERSONAL_RADIUS 0.1

#define MAX_BOID_VEL 1.75 

#define SEPERATION_GAIN 0.05 
#define ALIGNMENT_GAIN  0.001
#define COHESION_GAIN   0.001

void Boids_QT(ECS_Manager &world, double dt, QuadTree &qt);
void Boids_EdgeAvoidance(ECS_Manager &world, double dt);
