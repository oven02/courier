#include "path-gen.hpp"
namespace gen{
    
std::vector<float> spline(enum degree deg, std::vector<std::pair<float, float>> pts, int samples){
    std::vector<float> spline
    if (deg == TWO){
        for(float t = 0, t <= 1, t += 1.0f/samples){

        }

    }else if (deg == THREE){
        float x0 = pts[0][0];
        float y0 = pts[0][1];
        float x1 = pts[1][0];
        float y1 = pts[1][1];
        float x2 = pts[2][0];
        float y2 = pts[2][1];
        for(float i = 0, i <= 1, i += 1.0f/samples){
            spline.append((1-t)*((1-t)*x1+t*x0)+t*((1-t)*x0+t*x2), (1-t)*((1-t)*y1+t*y0)+t*((1-t)*y0+t*y2));
            
        }
        return spline;
    }else if (deg == FOUR){
        for(float i = 0, i <= 1, i += 1.0f/samples){
            
        }
    }else if (deg == FIVE){
        for(float i = 0, i <= 1, i += 1.0f/samples){
            
        }
    }
}
}