#include <iostream>
#include <vector>
#include <map>
#include <typeinfo>
#include <memory>
#include <array>

#include <math.h>

#include <raylib-cpp.hpp>

#include "Vector2D.hpp"
#include "./ECS/components/PositionZ1_comp.hpp"
#include "./ECS/components/Position_comp.hpp"
#include "./ECS/components/Velocity_comp.hpp"
#include "./ECS/components/Acceleration_comp.hpp"


#define SCREEN_WIDTH_IN_PIXELS  640
#define SCREEN_HEIGHT_IN_PIXELS 640

#define SCREEN_WIDTH_METERS  4
#define SCREEN_HEIGHT_METERS 4

#define WORLD_RADIUS (SCREEN_WIDTH_METERS/2)

#define OBJECT_RADIUS 0.05

#define TOTAL_SUBSTEPS 8

#define TOTAL_OBJECTS 10

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
        this->storage_container_index = 0;
    }

    void add_component(T component){
    
        this->storage_container[this->storage_container_index] = component;
        this->storage_container_index++;
    
    }

    T *get_component(int entity_id){

        T* start_ptr = this->storage_container.data();
        
        return &(start_ptr[entity_id]); 
    }

    size_t get_component_count(){
        return this->storage_container.size();    
    }
  
  private:
      std::array<T, MAX_ENTITIES> storage_container;
      size_t storage_container_index;

      std::map<int, size_t> map;  
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

  private:

    std::map<std::string, VComponentStorage*> T_to_comp_storage_Map;
    

};

static double world2screen_X(double x){
    return x * ((double)SCREEN_WIDTH_IN_PIXELS/SCREEN_WIDTH_METERS); 
}
static double world2screen_Y(double y){
    return y * -((double)SCREEN_HEIGHT_IN_PIXELS/SCREEN_HEIGHT_METERS) + (double)SCREEN_HEIGHT_IN_PIXELS;
}

void Physics_init(std::vector<Position_Component> &a){
    for (int i = 0; i < a.size(); i++){
        Vector2D temp_vec(2.0, 2.0);
        a[i].position = temp_vec;
    } 
}

void Physics_System(ECS_Mananger &world){

    for (int i = 0; i < world.get_component_count<Position_Component>(); i ++){
        Vector2D temp_pos = world.get_component<Position_Component>(i)->position;
        
        Vector2D new_pos  = world.get_component<Position_Component>(i)->position + 
                   TEMP_DT*(world.get_component<Velocity_Component>(i)->velocity) + 
                   TEMP_DT*TEMP_DT * (world.get_component<Acceleration_Component>(i)->accel);  
        
        world.get_component<Position_Component>(i)->position   = new_pos; 
        world.get_component<PositionZ1_Component>(i)->position = temp_pos;
        
        world.get_component<Velocity_Component>(i)->velocity = (1/TEMP_DT)*(world.get_component<Position_Component>(i)->position - 
                                                        world.get_component<PositionZ1_Component>(i)->position);
    }

};

void Render_init(){};
void Render_System(ECS_Mananger &world){
   
    BeginDrawing();
    ClearBackground(BLACK); 

    for (int i = 0; i < world.get_component_count<Position_Component>(); i ++){
        Vector2D obj_pos = world.get_component<Position_Component>(i)->position;  
        raylib::Vector2 temp_pos(world2screen_X(obj_pos.x),world2screen_Y(obj_pos.y) );
        temp_pos.DrawCircle(world2screen_X(OBJECT_RADIUS), BLUE);
           
    }

    DrawFPS(10,10);

    EndDrawing();  
    
}

void Bounds_System(ECS_Mananger &world){
    //for (auto it = world.pos_comps.begin(); it != world.pos_comps.end(); ++ it){
      for (int i = 0; i < world.get_component_count<Position_Component>(); i ++){
        Position_Component *pos_comp_ptr = world.get_component<Position_Component>(i);   
        if(pos_comp_ptr->position.x < 0){
            pos_comp_ptr->position.x = 0;
            world.get_component<Velocity_Component>(i)->velocity.x *= -0.75;
        }
        if(pos_comp_ptr->position.x > 4.0){
            pos_comp_ptr->position.x = 4.0;
            world.get_component<Velocity_Component>(i)->velocity.x *= -0.75;
        }
        if(pos_comp_ptr->position.y < 0){
            pos_comp_ptr->position.y = 0;
            world.get_component<Velocity_Component>(i)->velocity.y *= -0.75;
        }
        if(pos_comp_ptr->position.y > 4.0){
            pos_comp_ptr->position.y = 4.0;
            world.get_component<Velocity_Component>(i)->velocity.y *= -0.75;
        }
    }
}

static float get_randf(){
    return (float)(rand()) / (float)(RAND_MAX);
}




int main() {
    
    // Initialization
    raylib::Color textColor(LIGHTGRAY);
    raylib::Window w(SCREEN_WIDTH_IN_PIXELS, SCREEN_HEIGHT_IN_PIXELS, WINDOW_NAME);
    
    raylib::Rectangle box(10, GetScreenHeight()/2 - 50, 200, 100);

    SetTargetFPS(TARGET_FPS);

    //Create Arrays
   
    ECS_Mananger my_world;

    my_world.register_component<PositionZ1_Component>();
    my_world.register_component<Position_Component>();
    my_world.register_component<Velocity_Component>();
    my_world.register_component<Acceleration_Component>();
    

    for (int i = 0; i < 20; i++){
        int entity_id = i;
        PositionZ1_Component init_posz1_val = {entity_id, Vector2D(1.0, 2.0)};
        Position_Component init_pos_val     = {entity_id, Vector2D(1.0, 2.0)};
        Velocity_Component init_vel_val     = {entity_id, Vector2D(i*0.1, pow(i, 1.2)*0.1)};
        Acceleration_Component init_acc_val = {entity_id, Vector2D(0.0, -9.81)}; 
    
        my_world.add_component<PositionZ1_Component>(init_posz1_val);
        my_world.add_component<Position_Component>(init_pos_val);
        my_world.add_component<Velocity_Component>(init_vel_val);
        my_world.add_component<Acceleration_Component>(init_acc_val); 

    }

    
    // Main game loop
    while (!w.ShouldClose()) // Detect window close button or ESC key
    {
        // Update
        Physics_System(my_world); 
        Render_System(my_world);
        Bounds_System(my_world);
    }

    return 0;
}

