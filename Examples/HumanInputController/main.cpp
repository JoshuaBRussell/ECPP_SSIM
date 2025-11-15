#include <map>
#include <string>
#include <math.h>
#include <cmath>

#include "imgui.h"
#include "implot.h"
#include "rlImGui.h"

#include <Eigen/Dense>

#include "main.hpp"

#include "ECSManager.hpp"

#include "Newtonian_Sys.hpp"
#include "Gravity_Sys.hpp"
#include "Render.hpp"
#include "ParticleVisual.hpp"
#include "ConstraintVisual.hpp"
#include "Constraint.hpp"
#include "HumanInput_System.hpp"

#include "./../Examples/HumanInputController/Commands.hpp"

#include "./ECS/components/Rotation_comp.hpp"
#include "./ECS/components/Position_comp.hpp"
#include "./ECS/components/Velocity_comp.hpp"
#include "./ECS/components/Force_comp.hpp"
#include "./ECS/components/Torque_comp.hpp"
#include "./ECS/components/Rot_Inertia_comp.hpp"
#include "./ECS/components/Mass_comp.hpp"
#include "./ECS/components/Angular_Vel_comp.hpp"
#include "./ECS/components/Motion_comp.hpp"
#include "./ECS/components/Render_comp.hpp"
#include "./ECS/components/Collision_comp.hpp"
#include "./ECS/components/Vector_comp.hpp"
#include "./ECS/components/Particle_comp.hpp"
#include "./ECS/components/ODE_comp.hpp"
#include "./ECS/components/Constraint_comp.hpp"
#include "./ECS/components/Gravity_comp.hpp"
#include "./ECS/components/Connector_comp.hpp"
#include "./ECS/components/GUI_comp.hpp"

#define WORLD_RADIUS (SCREEN_WIDTH_METERS/2)

#define TOTAL_SUBSTEPS 8

#define TARGET_FPS 60

#define TEMP_DT (1/TARGET_FPS)

#define TWO_PI 6.2831853 // Only used for visualization modulo

#define WINDOW_NAME "Complex Shape Visualization"


static float x[180];

static float t = 0;
static int   i = 0;

// ---- Custom Plot Related ---- //
static std::map<int, float*> data_storage;

struct custom_plot_config {

};

void Custom_Plots_Init(ECS_Manager &world, struct custom_plot_config &custom_plot_config){
    
    for (auto it = world.get_component_begin<GUI_Component>();
              it < world.get_component_end<GUI_Component>(); it++){
         
        // Allocate Memory for a buffer
        float *f_buffer = new float[256];
        
        // Store Pointer to Buffers in Vector
        data_storage[it->entity_id] = f_buffer;
    }

    rlImGuiSetup(true);
}

void Custom_Plots_Shutdown(){
    rlImGuiShutdown(); // Not really needed, but included for 'completeness'
};

void Custom_Plots(ECS_Manager &world){
    
    rlImGuiBegin();
    ImPlot::CreateContext();

    bool open = true;
    bool* p_open = &open;
    
    // Prevent ImGui from saving a config state
    // Prefer to let the specific application set it
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = NULL;
    io.LogFilename = NULL;

    ImGui::SetNextWindowPos(ImVec2(0, 0));//, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(475, 350));//, ImGuiCond_FirstUseEver);
    
    ImGui::Begin("Joint Angles", p_open); 
    
    t += ImGui::GetIO().DeltaTime;
    i++; 
    
    if (ImPlot::BeginPlot("Line Plot")){
        ImPlot::SetupAxisLimits(ImAxis_X1,  0.0, 3.0); 
        ImPlot::SetupAxisLimits(ImAxis_Y1, -180, 180); 
        ImPlot::SetupAxes("x", "y"); 

        for (auto it = world.get_component_begin<GUI_Component>();
              it < world.get_component_end<GUI_Component>(); it++){

            int rb_id = it->entity_id;
            
            Rotation_Component* rot_comp_ptr = world.get_component<Rotation_Component>(rb_id);
            
             
            x[i%180] = std::fmod(i * 1.0/TARGET_FPS, 3.0); 
            
            float v = rot_comp_ptr->angle;
            while (v >= M_PI) v -= TWO_PI;
            while (v < M_PI)  v += TWO_PI;

            float *f_buffer = data_storage[it->entity_id];
            
            *(f_buffer +(i%180)) = (180.0/3.14159)*v - 360.0;
            
            ImPlot::PlotLine(("Angle " + std::to_string(it->entity_id)).c_str(), x, f_buffer,  i%180, 0, 0, sizeof(float));
        }
        
        ImPlot::EndPlot();
    } 
    
    ImPlot::DestroyContext();
    ImGui::End();
    
    rlImGuiEnd();

}


int main(){

    ECS_Manager my_world; 
    
    my_world.register_component<GUI_Component>(); 
    my_world.register_component<Render_Component>();
    my_world.register_component<Position_Component>();
    my_world.register_component<Velocity_Component>(); 
    my_world.register_component<Motion_Component>(); 
    my_world.register_component<Collision_Component>();
    my_world.register_component<Rotation_Component>();
    my_world.register_component<Vector_Component>(); 
    my_world.register_component<Particle_Component>(); 
    my_world.register_component<ODE_Component>();
    my_world.register_component<Force_Component>();
    my_world.register_component<Mass_Component>();
    my_world.register_component<Torque_Component>(); 
    my_world.register_component<Rot_Inertia_Component>();
    my_world.register_component<Angular_Vel_Component>(); 
    my_world.register_component<Gravity_Component>();
    my_world.register_component<Connector_Component>();
    my_world.register_component<Fixed_Rot_Component>(); 
    my_world.register_component<Relative_Rot_Component>(); // This isn't actually needed for this example
                                                           // but the program crashes if the component 
                                                           // isn't registered
    // Create the Background
    int bg_id = my_world.create_entity();
    Render_Component bg_render_comp      =  {bg_id, "./misc/background_w_grid.png",
                                              SCREEN_WIDTH_IN_PIXELS/2, SCREEN_HEIGHT_IN_PIXELS/2, 
                                              SCREEN_HEIGHT_IN_PIXELS , SCREEN_WIDTH_IN_PIXELS}; // x, y, h, w;
    Rotation_Component bg_rot_comp       = {bg_id, 0.0}; 
    my_world.add_component<Render_Component>(bg_render_comp);
    my_world.add_component<Rotation_Component>(bg_rot_comp); 
    
     
    // Initialize Systems after known established entites are created  
     
    // ---- Init Systems ---- //
    struct constr_visual_config constr_visual_config = {
        .screen_width_in_pixels  = SCREEN_WIDTH_IN_PIXELS,
        .screen_height_in_pixels = SCREEN_HEIGHT_IN_PIXELS,
        .screen_width_in_meters  = SCREEN_WIDTH_METERS,
        .screen_height_in_meters = SCREEN_HEIGHT_METERS
    };
    struct particle_visual_config particle_visual_config = {
        .screen_width_in_pixels  = SCREEN_WIDTH_IN_PIXELS,
        .screen_height_in_pixels = SCREEN_HEIGHT_IN_PIXELS,
        .screen_width_in_meters  = SCREEN_WIDTH_METERS,
        .screen_height_in_meters = SCREEN_HEIGHT_METERS
    }; 
    
    struct custom_plot_config custom_plot_config = {
    
    };

    struct render_config render_config = {
        .screen_width_in_pixels  = SCREEN_HEIGHT_IN_PIXELS,
        .screen_height_in_pixels = SCREEN_HEIGHT_IN_PIXELS,
        .window_title            = WINDOW_NAME, 
        .target_fps              = TARGET_FPS
    }; 
     
    
    // Create Command Objects 
    AddNewComplexObj *comm = new AddNewComplexObj;
    DeleteEntNearMouse *comm2 = new DeleteEntNearMouse;

    std::vector<struct key_action_pair> key_action_pairs = {
        {R_KEY, RELEASED, comm}
    };

    std::vector<struct button_action_pair> button_action_pairs = {
        {LEFT_MOUSE_BUTTON, PRESSED, comm2}
    };

    HumanInput_System_Init(my_world, key_action_pairs, button_action_pairs);

    Constraint_System_Init(my_world); 
    Render_System_Init(my_world, render_config); 

    Constraint_Visualization_Init(constr_visual_config);
    Particle_Visualization_Init(particle_visual_config); 
    Custom_Plots_Init(my_world, custom_plot_config); 
    
    
    // Move the Constraint World Coord. so it can be seen 
    Render_System_add_pre_render(Constraint_Visualization_System); 
    // Converts Physical Coordinates to something the Render_System can use (Screen Coords) 
    Render_System_add_pre_render(Particle_Visualization_System);
    
    //DearImGui GUI
    //Render_System_add_post_render(Custom_Plots);
    

    // Print basic instructions to console.
    std::cout << "\nINSTRUCTIONS: RELEASE R_KEY to add a complex looking rigid body object.\n";
    std::cout << "INSTRUCTIONS: PRESS LEFT_MOUSE_BUTTON to delete a rigid body." << std::endl;

    
    while (!Render_System_WindowShouldClose()) // Detect window close button or ESC key
    {
        for (int i = 0; i < 100; i ++){
            Gravity_System(my_world); 
            Constraint_System(my_world);
            Newtonian_System(my_world, GetFrameTime()/100);
            
        }
        
        Render_System(my_world); 
        
        HumanInput_System(my_world); 
         
    }
    
    Custom_Plots_Shutdown();
    Render_System_Shutdown();  
    
    return 0;
}

