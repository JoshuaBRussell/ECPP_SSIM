#include <cstddef>

template <class T>
class MemoryPool{
  private: 
    union FreeElement
    {
        T element;
        int next_free_item;
    }; 
    
    FreeElement *mem_pool_ptr;
    int next_free;
  public:
        //MemoryPool(size_t pool_count) 
        MemoryPool(){
            size_t pool_count = 2000;
            this->mem_pool_ptr = new FreeElement[pool_count]();
            this->next_free = 0;

            // Set the initial series of next values
            for (size_t i = 0; i < pool_count; i++){
                this->mem_pool_ptr[i].next_free_item = i+1;
            }
        }
     
        ~MemoryPool(){
            delete this->mem_pool_ptr;
        }

        int allocate(){ 
            int return_index = this->next_free;
            
            this->next_free = this->mem_pool_ptr[this->next_free].next_free_item;

            return return_index;
        }

        void deallocate(int index){
            this->mem_pool_ptr[index].next_free_item = this->next_free;
            this->next_free = index;
        }

        int insert(T item){
           int index = this->allocate();
           this->mem_pool_ptr[index].element = item;

           return index;
        }
        
        T& operator[](int index){
            return this->mem_pool_ptr[index].element; 
        }
};
