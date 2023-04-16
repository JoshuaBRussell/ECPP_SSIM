#include <iostream>
#include <vector>
#include <map>
#include <typeinfo>
#include <memory>
#include <array>
#include <set>
#include <string>

#include <math.h>

#include <raylib-cpp.hpp>

#include "Vector2D.hpp"
#include "./ECS/components/PositionZ1_comp.hpp"
#include "./ECS/components/Position_comp.hpp"
#include "./ECS/components/Velocity_comp.hpp"
#include "./ECS/components/Acceleration_comp.hpp"
#include "./ECS/components/Motion_comp.hpp"
#include "./ECS/components/Render_comp.hpp"
#include "./ECS/components/Boundary_comp.hpp"
#include "./ECS/components/Collision_comp.hpp"

#define SCREEN_WIDTH_IN_PIXELS  640
#define SCREEN_HEIGHT_IN_PIXELS 640

#define SCREEN_WIDTH_METERS  4
#define SCREEN_HEIGHT_METERS 4

#define WORLD_RADIUS (SCREEN_WIDTH_METERS/2)

#define OBJECT_RADIUS 0.05

#define TOTAL_SUBSTEPS 8

#define TOTAL_OBJECTS 50 

#define TARGET_FPS 60.0

#define TEMP_DT (1/TARGET_FPS)

#define WINDOW_NAME "Virtual Bob"

#define MAX_ENTITIES 1000

class VComponentStorage {
  
  public:
      virtual ~VComponentStorage() = default;
};

template <typename T>
class ComponentStorage : public VComponentStorage{
    
  public:

    ComponentStorage(){
        this->storage_container_count = 0;
    }

    void add_component(T component){
    
        this->storage_container[this->storage_container_count] = component;
        this->id_to_index_map.insert({component.entity_id, this->storage_container_count});
        this->storage_container_count++;
    
    }

    T *get_component(int entity_id){

        T* start_ptr = this->storage_container.data();
        
        return &(start_ptr[this->id_to_index_map[entity_id]]); 
    }

    size_t get_component_count(){
        return this->storage_container_count;    
    }

    T *begin(){
        return this->storage_container.data();
    }
    
    T *end(){
       return this->storage_container.data() + this->storage_container_count; 
    }
  
  private:
      std::array<T, MAX_ENTITIES> storage_container;
      size_t storage_container_count;

      std::map<int, size_t> id_to_index_map;  
};

class ECS_Mananger{

  public:
    
    ECS_Mananger(){

    }
    
    template<typename T> void register_component(){
        const char *type_name = typeid(T).name();
        
        ComponentStorage<T> *comp_storage_ptr = new ComponentStorage<T>;

        this->T_to_comp_storage_Map.insert({type_name, comp_storage_ptr});
        
    }
    
    template<typename T>
    void add_component(T component){
        const char *type_name = typeid(T).name();
        
        ComponentStorage<T>* my_ptr = static_cast<ComponentStorage<T>*>(this->T_to_comp_storage_Map[type_name]);
        my_ptr->add_component(component);
    }

    template<typename T>
    T *get_component(int entity_id){
        const char *type_name = typeid(T).name();

        ComponentStorage<T>* my_ptr = static_cast<ComponentStorage<T>*>(this->T_to_comp_storage_Map[type_name]);
        return my_ptr->get_component(entity_id);
    }

    template<typename T>
    size_t get_component_count(){
        const char *type_name = typeid(T).name();

        ComponentStorage<T>* my_ptr = static_cast<ComponentStorage<T>*>(this->T_to_comp_storage_Map[type_name]);
        return my_ptr->get_component_count(); 
    }

    template<typename T>
    T* get_component_begin(){
        const char *type_name = typeid(T).name();

        ComponentStorage<T>* my_ptr = static_cast<ComponentStorage<T>*>(this->T_to_comp_storage_Map[type_name]);

        return my_ptr->begin();
    }

    template<typename T>
    T* get_component_end(){
        const char *type_name = typeid(T).name();

        ComponentStorage<T>* my_ptr = static_cast<ComponentStorage<T>*>(this->T_to_comp_storage_Map[type_name]);

        return my_ptr->end();
    }

    int create_entity(){

        // loop through until a non-used ID is found
        int id_candidate = 0;
       
        //On the first call, container.begin() == container.end()
        //since it is empty. This means that 0 is a valid entry.
        //Any call after that, will be checked.
        std::set<int>::iterator it = this->id_container.begin();
        while (it != this->id_container.end()){
            
            id_candidate++;
            it = this->id_container.find(id_candidate);  
       
        }
        //Valid ID found. Insert into container
        this->id_container.insert(id_candidate);

        return id_candidate;
    }

  private:

    std::map<std::string, VComponentStorage*> T_to_comp_storage_Map;
    std::set<int> id_container; 

};

static double world2screen_X(double x){
    return x * ((double)SCREEN_WIDTH_IN_PIXELS/SCREEN_WIDTH_METERS); 
}
static double world2screen_Y(double y){
    return y * -((double)SCREEN_HEIGHT_IN_PIXELS/SCREEN_HEIGHT_METERS) + (double)SCREEN_HEIGHT_IN_PIXELS;
}

void Physics_init(std::vector<Position_Component> &a, double dt){
    for (int i = 0; i < a.size(); i++){
        Vector2D temp_vec(2.0, 2.0);
        a[i].position = temp_vec;
    } 
}

void Motion_System(ECS_Mananger &world, float dt){

    for (auto it = world.get_component_begin<Motion_Component>(); 
              it < world.get_component_end<Motion_Component>(); it++){ 
        
        Vector2D temp_pos = world.get_component<Position_Component>(it->entity_id)->position;
        
        /*
        Vector2D new_pos  = world.get_component<Position_Component>(it->entity_id)->position + 
                   dt*(world.get_component<Velocity_Component>(it->entity_id)->velocity) + 
                   dt*dt * (world.get_component<Acceleration_Component>(it->entity_id)->accel);  
        */
        /*
        Vector2D new_pos = 2*world.get_component<Position_Component>(it->entity_id)->position - 
                             world.get_component<PositionZ1_Component>(it->entity_id)->position +
                             dt*dt * (world.get_component<Acceleration_Component>(it->entity_id)->accel);   
        */

        Vector2D new_pos = world.get_component<Position_Component>(it->entity_id)->position +
                           dt*world.get_component<Velocity_Component>(it->entity_id)->velocity +
                           0.5*dt*dt*world.get_component<Acceleration_Component>(it->entity_id)->accel;
        world.get_component<Position_Component>(it->entity_id)->position   = new_pos; 
        world.get_component<PositionZ1_Component>(it->entity_id)->position = temp_pos;

        Vector2D new_vel = world.get_component<Velocity_Component>(it->entity_id)->velocity + 
                  dt*world.get_component<Acceleration_Component>(it->entity_id)->accel;
       
        world.get_component<Velocity_Component>(it->entity_id)->velocity = new_vel;
        //world.get_component<Velocity_Component>(it->entity_id)->velocity = (1/dt)*(world.get_component<Position_Component>(it->entity_id)->position - 
        //                                                world.get_component<PositionZ1_Component>(it->entity_id)->position);
    }

};

static std::map<std::string, raylib::Texture2D*> texture_repo;
void Render_init(){};

void Render_System(ECS_Mananger &world){
   
    BeginDrawing();
    ClearBackground(BLACK); 

    for (auto it = world.get_component_begin<Position_Component>(); 
              it < world.get_component_end<Position_Component>(); it++){
        
        // Check to see if the texture has been seen before
        std::string tex_loc = world.get_component<Render_Component>(it->entity_id)->texture_loc;
        if (texture_repo.find(tex_loc) == texture_repo.end()){
            // Add it to the texture texture_repo
            raylib::Texture2D* texture_ptr = new raylib::Texture2D("./misc/RedCirc.png");
            texture_repo.insert({tex_loc, texture_ptr}); 
        }
        
        Vector2D obj_pos = world.get_component<Position_Component>(it->entity_id)->position;
        
        raylib::Texture2D *texture_ptr = texture_repo[tex_loc];
        Vector2 tex_size = texture_ptr->GetSize();
        float scale_factor = ((float)world2screen_X(2*OBJECT_RADIUS))/tex_size.x; 
          
        raylib::Vector2 temp_pos(world2screen_X(obj_pos.x) - scale_factor*tex_size.x/2, 
                                 world2screen_Y(obj_pos.y) - scale_factor*tex_size.y/2); 
     
        texture_ptr->Draw(temp_pos, 0.0, scale_factor);
           
    }

    DrawFPS(10,10);

    EndDrawing();  
    
}

void Boundary_System(ECS_Mananger &world){
     
    for (auto it = world.get_component_begin<Boundary_Component>(); 
              it < world.get_component_end<Boundary_Component>(); it++){

        Position_Component *pos_comp_ptr = world.get_component<Position_Component>(it->entity_id);
        Collision_Component *coll_comp_ptr = world.get_component<Collision_Component>(it->entity_id);

        if((pos_comp_ptr->position.x  - coll_comp_ptr->radius)< 0){
            pos_comp_ptr->position.x = coll_comp_ptr->radius;
            world.get_component<Velocity_Component>(it->entity_id)->velocity.x *= -0.75;
        }
        if(pos_comp_ptr->position.x + coll_comp_ptr->radius> 4.0){
            pos_comp_ptr->position.x = 4.0 - coll_comp_ptr->radius;
            world.get_component<Velocity_Component>(it->entity_id)->velocity.x *= -0.75;
        }
        if((pos_comp_ptr->position.y - coll_comp_ptr->radius) < 0){
            pos_comp_ptr->position.y = coll_comp_ptr->radius;
            world.get_component<Velocity_Component>(it->entity_id)->velocity.y *= -0.75;
        }
        if(pos_comp_ptr->position.y + coll_comp_ptr->radius> 4.0){
            pos_comp_ptr->position.y = 4.0 - coll_comp_ptr->radius;
            world.get_component<Velocity_Component>(it->entity_id)->velocity.y *= -0.75;
        }
    }
}

void Collision_System(ECS_Mananger &world, double dt){

    for (auto coll_A = world.get_component_begin<Boundary_Component>(); 
              coll_A < world.get_component_end<Boundary_Component>(); coll_A++){

        Position_Component *pos_comp_A_ptr = world.get_component<Position_Component>(coll_A->entity_id);  
        for (auto coll_B = world.get_component_begin<Boundary_Component>(); 
              coll_B < world.get_component_end<Boundary_Component>(); coll_B++){
             
            if (coll_A != coll_B){
                Position_Component *pos_comp_B_ptr = world.get_component<Position_Component>(coll_B->entity_id);   
                // Find distance between to OBJECT_RADIUS
                Vector2D disp_vec = pos_comp_A_ptr->position - pos_comp_B_ptr->position;
                double dist = disp_vec.mag();
                // If the distance is less the sum of their radii -> collision occured
                
                if ( dist <= 2*OBJECT_RADIUS){
                    Vector2D norm_vec = (1/dist)*(disp_vec);

                    Vector2D temp = (2*OBJECT_RADIUS - dist) * norm_vec;
                    Vector2D prev_pos_A = pos_comp_A_ptr->position;
                    Vector2D prev_pos_B = pos_comp_B_ptr->position;
                    
                    pos_comp_A_ptr->position += temp;
                    pos_comp_B_ptr->position -= temp; 
                    
                    // Update Velocities
                     
                    Velocity_Component *vel_comp_A_ptr = world.get_component<Velocity_Component>(coll_A->entity_id);  
                    Velocity_Component *vel_comp_B_ptr = world.get_component<Velocity_Component>(coll_B->entity_id);
                    
                    Vector2D temp_vel = vel_comp_A_ptr->velocity;
                    vel_comp_A_ptr->velocity = vel_comp_B_ptr->velocity;
                    vel_comp_B_ptr->velocity = temp_vel; 
                    

                }
                
            }
        }
    }
}

static float get_randf(){
    return (float)(rand()) / (float)(RAND_MAX);
}

static void add_new_ball(ECS_Mananger &my_world){
    
    int entity_id = my_world.create_entity();
    
    PositionZ1_Component init_posz1_val = {entity_id, Vector2D(1.0, 2.0)};
    Position_Component init_pos_val     = {entity_id, Vector2D(1.0, 2.0)};
    Velocity_Component init_vel_val     = {entity_id, Vector2D(entity_id*0.1, pow(entity_id, 1.2)*0.1)};
    Acceleration_Component init_acc_val = {entity_id, Vector2D(0.0, -0.81)}; 
    
    Motion_Component init_mot_val       = {entity_id}; 
    Render_Component init_render_val    = {entity_id, "./misc/RedCirc.png"};
    Boundary_Component init_bounds_val  = {entity_id};
    Collision_Component init_coll_comp  = {entity_id, OBJECT_RADIUS};
    
    my_world.add_component<PositionZ1_Component>(init_posz1_val);
    my_world.add_component<Position_Component>(init_pos_val);
    my_world.add_component<Velocity_Component>(init_vel_val);
    my_world.add_component<Acceleration_Component>(init_acc_val); 

    my_world.add_component<Motion_Component>(init_mot_val);
    my_world.add_component<Render_Component>(init_render_val); 
    my_world.add_component<Boundary_Component>(init_bounds_val);
    my_world.add_component<Collision_Component>(init_coll_comp); 
}


int main() {
    
    // Initialization
    raylib::Color textColor(LIGHTGRAY);
    raylib::Window w(SCREEN_WIDTH_IN_PIXELS, SCREEN_HEIGHT_IN_PIXELS, WINDOW_NAME);
    raylib::Mouse Mouse;
    
    raylib::Rectangle box(10, GetScreenHeight()/2 - 50, 200, 100);

    
    SetTargetFPS(TARGET_FPS);

    //Create Arrays
   
    ECS_Mananger my_world;
    
    // Indicator Components - should these be 'archetypes'?
    my_world.register_component<Motion_Component>();
    my_world.register_component<Render_Component>();
    my_world.register_component<Boundary_Component>();
    my_world.register_component<Collision_Component>();

    my_world.register_component<PositionZ1_Component>();
    my_world.register_component<Position_Component>();
    my_world.register_component<Velocity_Component>();
    my_world.register_component<Acceleration_Component>();
    

    for (int entity_id = 0; entity_id < 5; entity_id++){
        add_new_ball(my_world);
    }
    raylib::Texture2D red_texture("./misc/RedCirc.png");
    raylib::Texture2D blue_texture("./misc/BlueCirc.png");
    raylib::Texture2D* tex = new raylib::Texture2D("./misc/RedCirc.png"); 
    
    std::map<std::string, raylib::Texture2D*> a;
    a.insert({"Red", &red_texture});
    a.insert({"Blue", &blue_texture});
    a.insert({"Red", &red_texture});
    a.insert({"NotRed", tex});
    

    std::map<std::string, raylib::Texture2D*>::iterator it;
    for (it = a.begin(); it != a.end(); ++it){
        std::cout << it->first << std::endl;
    }
    // Main game loop
    while (!w.ShouldClose()) // Detect window close button or ESC key
    {
        //BeginDrawing();
        //Vector2 loc = {320-32, 320 - 32};
        //red_texture.Draw(loc, 0.0, 0.5);
        //EndDrawing();

        
        if (Mouse.IsButtonPressed(0)){

            add_new_ball(my_world);
        }
        // Update
        for (int i = 0; i < 8; i++){
            
            Motion_System(my_world, TEMP_DT/8); 
            Collision_System(my_world, TEMP_DT/8);  
            Boundary_System(my_world); 
        }

        Render_System(my_world);
        
    }

    return 0;
}

