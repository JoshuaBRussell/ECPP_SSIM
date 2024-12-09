#include "RigidBodyUtil.hpp"

#include "./ECS/components/Rotation_comp.hpp"
#include "./ECS/components/Position_comp.hpp"
#include "./ECS/components/Velocity_comp.hpp"
#include "./ECS/components/Force_comp.hpp"
#include "./ECS/components/Torque_comp.hpp"
#include "./ECS/components/Rot_Inertia_comp.hpp"
#include "./ECS/components/Mass_comp.hpp"
#include "./ECS/components/Angular_Vel_comp.hpp"
#include "./ECS/components/Render_comp.hpp"
#include "./ECS/components/Particle_comp.hpp"
#include "./ECS/components/ODE_comp.hpp"
#include "./ECS/components/Constraint_comp.hpp"
#include "./ECS/components/Gravity_comp.hpp"
#include "./ECS/components/GUI_comp.hpp"




void add_rigid_body_to_world(ECS_Manager &world, int entity_id, Eigen::Vector2d pos, double angle){
    
    Particle_Component particle_flag      = {entity_id};
    Position_Component particle_pos       = {entity_id, pos};
    Velocity_Component particle_vel       = {entity_id, Eigen::Vector2d(0.0, 0.0)}; 
    Rotation_Component rot_val            = {entity_id, angle}; 
    Render_Component render_val           = {entity_id, "./misc/black_square.png",
                                                         0,   0,
                                                        50, 200}; // x, y, h, w; 
    GUI_Component      gui_flag           = {entity_id}; 
    
    ODE_Component ode_val                 = {entity_id, INT_METHOD::RK4}; 
    Force_Component force_val             = {entity_id, Eigen::Vector2d(0.0, 0.0)};
    Mass_Component mass_val               = {entity_id, 1.0}; 
    Gravity_Component grav_val            = {entity_id}; 
    Torque_Component torque_val           = {entity_id, 0.0}; 
    Rot_Inertia_Component rot_inertia_val = {entity_id, 1.0};
    Angular_Vel_Component rot_vel_val     = {entity_id, 0.0};

    world.add_component<Particle_Component>(particle_flag);
    world.add_component<Position_Component>(particle_pos);
    world.add_component<Velocity_Component>(particle_vel);
    world.add_component<Render_Component>(render_val);
    world.add_component<GUI_Component>(gui_flag); 
    world.add_component<Rotation_Component>(rot_val); 
    world.add_component<ODE_Component>(ode_val);
    world.add_component<Force_Component>(force_val); 
    world.add_component<Mass_Component>(mass_val); 
    world.add_component<Gravity_Component>(grav_val); 
    world.add_component<Torque_Component>(torque_val);
    world.add_component<Rot_Inertia_Component>(rot_inertia_val); 
    world.add_component<Angular_Vel_Component>(rot_vel_val);    
    
}

void add_fixed_pos_constr(ECS_Manager &world, 
                          int entity_id, int rb_id, 
                          Eigen::Vector2d world_pos, Eigen::Vector2d rel_pos){

    Fixed_Rot_Component fixed_rot_constr = {entity_id, rb_id, 
                                            world_pos, // world space point 
                                            rel_pos,  // body space  
                                            0.0}; 
    Render_Component init_constr_rend       = {entity_id, "./misc/blue_circle.png",
                                                           0,  0, 
                                                          15, 15}; 
    Position_Component init_constr_pos      = {entity_id, world_pos}; 
    Particle_Component init_particle_flag   = {entity_id}; 
    Rotation_Component init_constr_rot_val  = {entity_id, 1.5708};

    world.add_component<Fixed_Rot_Component>(fixed_rot_constr);
    world.add_component<Position_Component>(init_constr_pos); 
    world.add_component<Particle_Component>(init_particle_flag); 
    world.add_component<Render_Component>(init_constr_rend);
    world.add_component<Rotation_Component>(init_constr_rot_val);

}

void add_rel_constr(ECS_Manager &world, 
                     int entity_id, int rb1_id, int rb2_id, 
                     Eigen::Vector2d rel_pos1, Eigen::Vector2d rel_pos2, Eigen::Vector2d init_pos){
  
    Relative_Rot_Component rel_rot_constr = {entity_id, rb1_id, rb2_id, 
                                            rel_pos1, // body space - rigid body 1 
                                            rel_pos2, // body space - rigid body 2 
                                            0.0}; 
    Render_Component init_constr_rend2      = {entity_id, "./misc/blue_circle.png",
                                               0,  0,
                                              15, 15}; 
    Position_Component init_constr_pos2     = {entity_id, init_pos}; 
    Particle_Component init_particle_flag4  = {entity_id}; 
    Rotation_Component init_constr_rot_val2     = {entity_id, 1.5708};

    world.add_component<Relative_Rot_Component>(rel_rot_constr);
    world.add_component<Position_Component>(init_constr_pos2); 
    world.add_component<Particle_Component>(init_particle_flag4); 
    world.add_component<Render_Component>(init_constr_rend2);
    world.add_component<Rotation_Component>(init_constr_rot_val2);

}


void add_complex_shape_to_world(ECS_Manager &world){
    
    int rb1_id = -1;
    int rb2_id = -1;
    int rb3_id = -1;
    int rb4_id = -1;
    int rb5_id = -1; 

    // First Rigid Body
    rb1_id = world.create_entity();
    add_rigid_body_to_world(world, rb1_id, Eigen::Vector2d(1.0, 0.0), 1.5707);
    
    // Second Rigid Body
    rb2_id = world.create_entity(); 
    add_rigid_body_to_world(world, rb2_id, Eigen::Vector2d(2.0, -1.0), 0.0); 
    
    // Third Rigid Body
    rb3_id = world.create_entity(); 
    add_rigid_body_to_world(world, rb3_id, Eigen::Vector2d(1.0, -2.0), 1.5707); 

    // Fourth Rigid Body
    rb4_id = world.create_entity(); 
    add_rigid_body_to_world(world, rb4_id, Eigen::Vector2d(0.0, -1.0), 0.0); 

    // Fifth Rigid Body
    rb5_id = world.create_entity(); 
    add_rigid_body_to_world(world, rb5_id, Eigen::Vector2d(1.0, -1.0), 0.785397); 

    // Fixed Position Constraint #1
    int fixed_constr_id = world.create_entity();
    add_fixed_pos_constr(world, 
                         fixed_constr_id, rb1_id, 
                         Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 1.0)); 
    
    // Fixed Position Constraint #2 
    fixed_constr_id = world.create_entity();
    add_fixed_pos_constr(world, 
                         fixed_constr_id, rb4_id, 
                         Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 1.0)); 
    
    // Relative Position Constraint #1
    int rel_constr_id = world.create_entity();
    add_rel_constr(world, rel_constr_id, rb1_id, rb2_id,
                    Eigen::Vector2d(0.0, -1.0), Eigen::Vector2d(0.0, 1.0), Eigen::Vector2d(0.0, 1.0));
     
    // Relative Position Constraint #2
    rel_constr_id = world.create_entity();
    add_rel_constr(world, rel_constr_id, rb2_id, rb3_id,
                    Eigen::Vector2d(0.0, -1.0), Eigen::Vector2d(0.0, -1.0), Eigen::Vector2d(0.0, 1.0));
      
    // Relative Position Constraint #3
    rel_constr_id = world.create_entity();
    add_rel_constr(world, rel_constr_id, rb3_id, rb4_id,
                    Eigen::Vector2d(0.0, 1.0), Eigen::Vector2d(0.0, -1.0), Eigen::Vector2d(0.0, 1.0));

    // Relative Position Constraint #4
    rel_constr_id = world.create_entity();
    add_rel_constr(world, rel_constr_id, rb4_id, rb5_id,
                    Eigen::Vector2d(0.0, 1.0), Eigen::Vector2d(0.0, 1.414213), Eigen::Vector2d(0.0, 0.0));
      
    // Relative Position Constraint #5
    rel_constr_id = world.create_entity();
    add_rel_constr(world, rel_constr_id, rb2_id, rb5_id,
                    Eigen::Vector2d(0.0, -1.0), Eigen::Vector2d(0.0, -1.414213), Eigen::Vector2d(2.0, -2.0));
    
}
