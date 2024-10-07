#pragma once

#include <stddef.h>
#include <map>
#include <unordered_map>
#include <array>
#include <assert.h>

#include "ECS.hpp"

class VComponentStorage {
  
  public:
      virtual ~VComponentStorage() = default;
      virtual void delete_component(int entity_id) = 0;
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
            
        return_result =  this->storage_container.data() + this->id_to_index_map.at(entity_id);

        return return_result;
    }

    void delete_component(int entity_id) override {
        
        // If it even exist
        auto it = this->id_to_index_map.find(entity_id);
        if( it != this->id_to_index_map.end()){
            
            // Take Last Component And Overwrite Deleted One If There is More Than One
            // This keeps things contiguous
            if (this->storage_container_count > 1){
                // Find the ID mapped to the last element in the list
                //TODO: This is a slow way to find this
                int last_element_id = -1; 
                for (auto it = this->id_to_index_map.begin(); it != this->id_to_index_map.end(); it++){
                    if (it->second == this->storage_container_count-1){
                        last_element_id = it->first;
                        break;
                    }
                }
                assert(last_element_id != -1);

                this->storage_container[it->second] = this->storage_container[this->storage_container_count-1];
                this->id_to_index_map.insert({last_element_id, it->second});   
            }

            this->storage_container_count--;
            
            this->id_to_index_map.erase(it);
        }
        
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

      std::unordered_map<int, size_t> id_to_index_map;  
};

