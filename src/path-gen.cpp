#include "path-gen.hpp"
namespace gen{
    
std::vector<float> spline(enum degree deg, std::vector<std::pair<float, float>> pts, int samples){
    std::vector<float> spline
    if (deg == TWO){
        float x0 = pts[0][0];
        float y0 = pts[0][1];
        float x1 = pts[1][0];
        float y1 = pts[1][1];
        for(float t = 0, t <= 1, t += 1.0f/samples){
            spline.append((1-t)*x0+t*x1, (1-t)*x0+t*x1);
        }
    return spline;

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
        float x0 = pts[0][0];
        float y0 = pts[0][1];
        float x1 = pts[1][0];
        float y1 = pts[1][1];
        float x2 = pts[2][0];
        float y2 = pts[2][1];
        float x3 = pts[2][0];
        float y3 = pts[2][1];
        for(float i = 0, i <= 1, i += 1.0f/samples){
            spline.append(((1-t)**3)*x0 + 3*((1-t)**2)*t*x1 + 3*((1-t))*(t**2)*x2 + (t**3)*x3, ((1-t)**3)*y0 + 3*((1-t)**2)*t*y1 + 3*((1-t))*(t**2)*y2 + (t**3)*y3);
            
        }
        return spline;
    }else if (deg == FIVE){
        for(float i = 0, i <= 1, i += 1.0f/samples){
            
        }
    }
}
}