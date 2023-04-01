#pragma once

#include <iostream>

class Vector2D {
  public:
    double x;
    double y;


    Vector2D() {
        this->x = 0.0;
        this->y = 0.0;
    }

    Vector2D(double x, double y){
        this->x = x;
        this->y = y;
    }

    void print(){ std::cout << "(" << this->x << "," << this->y << ")" << std::endl; };


    Vector2D operator+(const Vector2D& vec){
        Vector2D result;
        result.x = this->x + vec.x;
        result.y = this->y + vec.y;
        
        return result;
    }

    Vector2D operator-(const Vector2D& vec){
        Vector2D result;
        result.x = this->x - vec.x;
        result.y = this->y - vec.y;

        return result;
    }

    Vector2D operator=(const Vector2D& vec){
        
        this->x = vec.x;
        this->y = vec.y;
        return *this;
    }

    Vector2D operator*=(const double& s){
        
        this->x = s*this->x;
        this->y = s*this->y;
        return *this;
    } 
};


Vector2D operator*(const double& s, Vector2D vec){
        Vector2D result;
        result.x = s*vec.x;
        result.y = s*vec.y;
        
        return result;
}


