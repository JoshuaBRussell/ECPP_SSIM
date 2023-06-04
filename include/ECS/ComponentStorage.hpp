#pragma once

#include <stddef.h>
#include <map>
#include <array>

#include "ECS.hpp"

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
        
        T *return_result = nullptr;
        if (this->id_to_index_map.count(entity_id)){
            
            T *start_ptr = this->storage_container.data(); 
            return_result = &(start_ptr[this->id_to_index_map.at(entity_id)]); 
        
        }

        return return_result;
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

