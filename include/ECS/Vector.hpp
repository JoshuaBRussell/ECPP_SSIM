#pragma once

#include <iostream>
#include <math.h>

template<unsigned int N>
class Vector {
  public:

    double x[N] = {0.0};
    
    Vector() {
    
    }

    Vector(double *elements, int len){
        for (int i = 0; i < len; i++){
            x[i] = elements[i]; 
        }

    }

    void print(){
        for (unsigned int i = 0; i < N; i++){
            std::cout << "[" << i << "]: " << x[i] << std::endl; 
        }
    }
    
    double mag(){
        double mag_squared = 0.0; 
        for (unsigned int i = 0; i < N; i++){
            mag_squared += x[i]*x[i]; 
        } 

        return std::sqrt(mag_squared);
    }

    Vector operator+(const Vector<N>& vec){
        Vector<N> result;
        for (unsigned int i = 0; i < N; i++){
            result[i] = x[i] + vec.x[i]; 
        }

        return result;
    }
          
    double& operator[](std::size_t idx)       { return x[idx]; }
    const double& operator[](std::size_t idx) const { return x[idx]; }

/*
    Vector operator-(const Vector& vec){
        Vector result;
        result.x = this->x - vec.x;
        result.y = this->y - vec.y;

        return result;
    }

    Vector operator=(const Vector& vec){
        
        this->x = vec.x;
        this->y = vec.y;
        return *this;
    }
    
    Vector operator+=(const Vector& vec){
        
        this->x = this->x + vec.x;
        this->y = this->y + vec.y;
        return *this;
    }

    Vector operator-=(const Vector& vec){
        
        this->x = this->x - vec.x;
        this->y = this->y - vec.y;
        return *this;
    }

    Vector operator*=(const double& s){
        
        this->x = s*this->x;
        this->y = s*this->y;
        return *this;
    }
*/   
};

//template <unsigned int N>
//Vector<N> operator*(const double& s, Vector<N> vec);

template <unsigned int N>
Vector<N> operator*(const double& s, Vector<N> vec){
        Vector<N> result;
        
        for (unsigned int i = 0; i < N; i++){
            result[i] = s*vec[i];
        }

        return result;
}
