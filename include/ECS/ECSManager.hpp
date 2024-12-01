#pragma once

#include <string>
#include <typeinfo>
#include <vector>
#include <map>
#include <set>
#include <cassert>
#include <iostream>
#include <algorithm>
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

    std::set<int> get_entity_refs(int entity_id){
        std::set<int> entity_refs;
        for (auto it = this->entity_refs_callbacks.begin(); it != this->entity_refs_callbacks.end(); it++){
            std::set<int> d = (**it)(entity_id, *this);
            std::merge(entity_refs.begin(), entity_refs.end(), d.begin(),
            d.end(), inserter(entity_refs, entity_refs.begin()));
        }

        return entity_refs;
    }

    void delete_entity(int entity_id){
        
        std::cout << "Deleting: " << entity_id << "\n"; 
        
        for (auto it = this->T_to_comp_storage_Map.begin(); it != this->T_to_comp_storage_Map.end(); it++){
            // Check to see if a deletion occurs. Since there isn't a cache of what entity has what component (intentionally),
            // this lets us know if there was a component deleted that was assigned to an entity
            
             
            if(it->second->delete_component(entity_id)){

                std::cout << "Checking E of Comp: " << it->first << std::endl; 
                auto search_result = this->comp_to_sys_comp_change_callbacks.find(it->first); 
                if (search_result != this->comp_to_sys_comp_change_callbacks.end()){ 
                    std::cout << "Component " << it->first << " has a callback.\n"; 
                    // Rather than invoking the system callback every time a component is deleted that the system 
                    // is interested in, note that it was invoked and defer calling it until the end of the 
                    // function. This is to avoid multiple calls to potentially expensive callbacks.
                    //
                    // This also ensures all components for an entity are deleted before a callback potentially 
                    // uses that entity with missing components - before it is completely deleted.
                    std::vector<void (*)(ECS_Manager&)> *v = this->comp_to_sys_comp_change_callbacks[it->first];
                    for (auto cb_ptr = v->begin(); cb_ptr != v->end(); cb_ptr++){
                        this->sys_comp_change_callbacks_set.insert(*cb_ptr); 
                    }
                }
            } 

        }

        this->id_container.erase(entity_id); 
        
    }
    
    void destroy_entity(int entity_id){ 
        

        // TODO: This probably doesn't handle certain entity reference trees very well.
        // Think about this more
        std::set<int> e = this->get_entity_refs(entity_id); 
        this->delete_entity(entity_id);

        for (auto it = e.begin(); it  != e.end(); it++){
            this->delete_entity(*it);
        }

        for (auto it = this->sys_comp_change_callbacks_set.begin(); it != this->sys_comp_change_callbacks_set.end(); it++){
            (**it)(*this);
        }

        this->sys_comp_change_callbacks_set.clear();        
    }

    bool does_entity_exist(int entity_id){
        return this->id_container.find(entity_id) != this->id_container.end();
    }
    
    template <typename T>
    void augmentation_callback(void (*sys_callback)(ECS_Manager &world)){
        
        const char *type_name = typeid(T).name();
        std::vector<void (*)(ECS_Manager&)> *v;
        
        auto search_result = this->comp_to_sys_comp_change_callbacks.find(type_name); 
        if (search_result == this->comp_to_sys_comp_change_callbacks.end()){
            v = new std::vector<void (*)(ECS_Manager &)>;
        } else {
            v = search_result->second; 
        }

        v->push_back(sys_callback);

        this->comp_to_sys_comp_change_callbacks.insert({type_name, v});

    }

    void set_entity_refs_callbacks(std::set<int> (*callback)(int entity_id, ECS_Manager &world)){
        this->entity_refs_callbacks.insert(callback); 
    }

  private:
    
    std::map<std::string, VComponentStorage*> T_to_comp_storage_Map;
    std::set<int> id_container;
    std::set<void (*)(ECS_Manager &)> sys_comp_change_callbacks_set;
    std::map<std::string, std::vector<void (*)(ECS_Manager &)>*> comp_to_sys_comp_change_callbacks; 
    std::set<std::set<int> (*)(int entity_id, ECS_Manager &world)> entity_refs_callbacks;
};
