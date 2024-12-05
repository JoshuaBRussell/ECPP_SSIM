#pragma once

#include <stddef.h>
#include <map>
#include <iostream>
#include <unordered_map>
#include <array>
#include <assert.h>

#include "ECS.hpp"

class VComponentStorage {
  
  public:
      virtual ~VComponentStorage() = default;
      virtual bool delete_component(int entity_id) = 0;
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

    void print_storage(){
        std::cout << "{ ";
        
        // I think type issues were making it so if storage_container_count == 0, it was causing it to 
        // to always appear as less than 0 - so it just kept looping.
        // I don't feel like figuring this out properly right now, so I am placing a conditional. 
        if (this->storage_container_count != 0){
            for (size_t i = 0; i < this->storage_container_count - 1; i ++){
                std::cout << "[" << i << "] : " << this->storage_container[i].entity_id << ", ";
            }
        }
        
        if (this->storage_container_count > 1){
            size_t last_index = this->storage_container_count - 1;
            std::cout << "[" << last_index << "] : " << this->storage_container[last_index].entity_id;
        }

        std::cout << "}" << std::endl;
    }
    
    // In the event that a component was found and subsequently deleted, this returns true
    bool delete_component(int entity_id) override {
        
        bool comp_deleted = false; 
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
                
                // Move the last element to the deleted items location
                this->storage_container[it->second] = this->storage_container[this->storage_container_count-1];
                this->id_to_index_map[last_element_id] =  it->second; // The last element is known to already exist at this time,
                                                                      // so this shouldn't accidently create anything new
            }

            this->storage_container_count--;
            this->id_to_index_map.erase(it);
        
            comp_deleted = true;
        }
       
        return comp_deleted;
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

