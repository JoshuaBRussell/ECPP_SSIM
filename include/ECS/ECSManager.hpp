#pragma once

#include <string>
#include <typeinfo>
#include <map>
#include <set>
#include <cassert>
#include <iostream>

#include "ComponentStorage.hpp"

class ECS_Manager{

  public:
    
    ECS_Manager(){

    }
    
    template<typename T> void register_component(){
        const char *type_name = typeid(T).name();
        
        ComponentStorage<T> *comp_storage_ptr = new ComponentStorage<T>;

        this->T_to_comp_storage_Map.insert({type_name, comp_storage_ptr});
        
    }
    
    template<typename T>
    void add_component(T component){
        const char *type_name = typeid(T).name();
        
        int registered_count = this->T_to_comp_storage_Map.count(type_name);
        if (registered_count == 0){
            std::cerr << "[ERROR]: Cannot add component. It has not been registered." << std::endl;
            assert(registered_count > 0); 
        }
        
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
