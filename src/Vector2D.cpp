#include "Vector2D.hpp"

Vector2D operator*(const double& s, Vector2D vec){
        Vector2D result;
        result.x = s*vec.x;
        result.y = s*vec.y;
        
        return result;
}
