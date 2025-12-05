#include "ascentLib/path-gen.hpp"
#include <cmath>


namespace gen{
    
std::vector<std::pair<float, float>> spline(enum degree deg, std::vector<std::pair<float, float>> pts, int samples){
    std::vector<std::pair<float, float>> out;
    if (deg == TWO){
        float x0 = pts[0].first;
        float y0 = pts[0].second;
        float x1 = pts[1].first;
        float y1 = pts[1].second;
        for(float t = 0; t <= 1; t += 1.0f/samples){
            out.push_back({(1-t)*x0+t*x1, (1-t)*y0+t*y1});
        }
    return out;

    }else if (deg == THREE){
        float x0 = pts[0].first;
        float y0 = pts[0].second;
        float x1 = pts[1].first;
        float y1 = pts[1].second;
        float x2 = pts[2].first;
        float y2 = pts[2].second;
        for(float t = 0; t <= 1; t += 1.0f/samples){
            out.push_back({(1-t)*((1-t)*x1+t*x0)+t*((1-t)*x0+t*x2), (1-t)*((1-t)*y1+t*y0)+t*((1-t)*y0+t*y2)});
            
        }
        return out;
    }else if (deg == FOUR){
        float x0 = pts[0].first;
        float y0 = pts[0].second;
        float x1 = pts[1].first;
        float y1 = pts[1].second;
        float x2 = pts[2].first;
        float y2 = pts[2].second;
        float x3 = pts[2].first;
        float y3 = pts[2].second;
        for(float t = 0; t <= 1; t += 1.0f/samples){
            out.push_back({(std::pow((1-t),3))*x0 + 3*std::pow((1-t),2)*t*x1 + 3*((1-t))*std::pow(t, 2)*x2 + std::pow(t, 3)*x3, std::pow((1-t), 3)*y0 + 3*std::pow((1-t), 2)*t*y1 + 3*((1-t))*std::pow(t, 2)*y2 + std::pow(t, 3)*y3});
            
        }
        return out;
    }else if (deg == FIVE){
        float x0 = pts[0].first;
        float y0 = pts[0].second;
        float x1 = pts[1].first;
        float y1 = pts[1].second;
        float x2 = pts[2].first;
        float y2 = pts[2].second;
        float x3 = pts[2].first;
        float y3 = pts[2].second;
        float x4 = pts[3].first;
        float y4 = pts[3].second;
        for(float t = 0; t <= 1; t += 1.0f/samples){
            out.push_back({std::pow(1-t,4)*x0+4*std::pow(1-t, 3)*t*x1+6*std::pow(1-t, 2)*std::pow(t, 2)*x2+4*(1-t)*std::pow(t,3)*x3+std::pow(t,4)*x4, std::pow(1-t,4)*y0+4*std::pow(1-t, 3)*t*y1+6*std::pow(1-t, 2)*std::pow(t, 2)*y2+4*(1-t)*std::pow(t,3)*y3+std::pow(t,4)*y4});
        }
    }
    return {{0,0}};
}
}