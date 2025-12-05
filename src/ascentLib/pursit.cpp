#include <cmath>
#include <utility>
#include <vector>
#include "ascentLib/pursit.hpp"

float pt_to_pt_distance (Point pt1,Point pt2){
    float distance = std::sqrt(std::pow((pt2.x - pt1.x), 2) + std::pow((pt2.y - pt1.y),2));
    return distance;
}

float pt_to_pt_distance (Point pt1,std::pair<float, float> pt2){
    float distance = std::sqrt(std::pow((pt2.first - pt1.x), 2) + std::pow((pt2.second - pt1.y),2));
    return distance;
}

float sgn (float num){
  if (num >= 0){
    return 1;
  }else{
    return -1;
  }
}

Point currentPos = Point(0, 0);
float currentHeading = 0;
float lastFoundIndex = 0;
float lookAheadDis = 0.8;
float linearVel = 100;


bool using_rotation = false;




std::vector<float> pure_pursuit_step (std::vector<std::pair<float,float>> path, Point currentPos, float currentHeading, float lookAheadDis, float LFindex){

  float currentX = currentPos.x;
  float currentY = currentPos.y;
  Point goalPt = Point(0,0);

  lastFoundIndex = LFindex;
  bool intersectFound = false;

  for (float i = lastFoundIndex; path.size()-1 > i; i++){

    float x1 = path[i].first - currentX;
    float y1 = path[i].second - currentY;
    float x2 = path[i+1].first - currentX;
    float y2 = path[i+1].second - currentY;
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dr = std::sqrt (std::pow(dx,2) + std::pow(dy,2));
    float D = x1*y2 - x2*y1;
    float discriminant = std::pow(lookAheadDis,2) * std::pow(dr,2) - std::pow(D,2);

    if (discriminant >= 0){
      float sol_x1 = (D * dy + sgn(dy) * dx * std::sqrt(discriminant)) / std::pow(dr,2);
      float sol_x2 = (D * dy - sgn(dy) * dx * std::sqrt(discriminant)) / std::pow(dr,2);
      float sol_y1 = (- D * dx + std::abs(dy) * std::sqrt(discriminant)) / std::pow(dr,2);
      float sol_y2 = (- D * dx - std::abs(dy) * std::sqrt(discriminant)) / std::pow(dr,2);

      Point sol_pt1 = Point(sol_x1 + currentX, sol_y1 + currentY);
      Point sol_pt2 = Point(sol_x2 + currentX, sol_y2 + currentY);

      
      float minX = fmin(path[i].first, path[i+1].first);
      float minY = fmin(path[i].second, path[i+1].second);
      float maxX = fmax(path[i].first, path[i+1].first);
      float maxY = fmax(path[i].second, path[i+1].second);

      if (((minX <= sol_pt1.x <= maxX) && (minY <= sol_pt1.y <= maxY)) || ((minX <= sol_pt2.x <= maxX) && (minY <= sol_pt2.y <= maxY))){
        intersectFound = true;

        if (((minX <= sol_pt1.x <= maxX) && (minY <= sol_pt1.y <= maxY)) && ((minX <= sol_pt2.x <= maxX) && (minY <= sol_pt2.y <= maxY))){
      
          if (pt_to_pt_distance(sol_pt1, path[i+1]) < pt_to_pt_distance(sol_pt2, path[i+1])){
            goalPt = sol_pt1;
          }else{
            goalPt = sol_pt2;
          }

        }else{
          
          if ((minX <= sol_pt1.x <= maxX) && (minY <= sol_pt1.y <= maxY)){
            goalPt = sol_pt1;
          }else{
            goalPt = sol_pt2;
          }
        }
       
        if (pt_to_pt_distance(goalPt, path[i+1]) < pt_to_pt_distance(Point (currentX, currentY), path[i+1])){
          lastFoundIndex = i;
          break;
        }else{
          lastFoundIndex = i+1;
        }

      }else{
        intersectFound = false;
        goalPt = Point (path[lastFoundIndex].first, path[lastFoundIndex].second);
      }
    }

  }
   

  float Kp = 3;

  float absTargetAngle = std::atan2 (goalPt.y-currentPos.y, goalPt.x-currentPos.x) *180/M_PI;
  if (absTargetAngle < 0){ absTargetAngle += 360;}

  float turnError = absTargetAngle - currentHeading;
  if(turnError > 180 or turnError < -180){
    turnError = -1 * sgn(turnError) * (360 - std::abs(turnError));
  }

  float turnVel = Kp*turnError;
  
  return {goalPt.x, goalPt.y, lastFoundIndex, turnVel};
}