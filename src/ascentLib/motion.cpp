#include "main.h" // IWYU pragma: keep
#include "ascentLib/odom.hpp"
#include "ascentLib/motion.hpp"
#include "ascentLib/util.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include <iostream> 
#include <cstdlib> // For integer abs()
#include <cmath>
#include <utility>
#include <algorithm> 


float right_out = 0;
float left_out = 0;
float dist;

chassis* mainChassis;

int sgn(float val){
  if (val > 0){
    return 1;
  }else if(val < 0){
    return -1;
  }else{
    return 0;
  }
}

double angleWraper(double value){
  double wrapped = fmod(value + 180.0, 360.0);
  if (wrapped < 0) {
      wrapped += 360.0;
  }
  return wrapped - 180.0;
}

float outA;
float outL; 


PID::PID(float inkP, float inkI, float inkD){
        kP = inkP;
        kI = inkI;
        kD = inkD;
}

void PID::changeVals(float inkP, float inkI, float inkD){
        kP = inkP;
        kI = inkI;
        kD = inkD;
}

    float PID::update(float sig){
        float error = sig;
            
        if (start == 0){
          prevError = error;
          start = 1;
        }
        integral += error;
        derivative = error - prevError;
        float output =  kP * error + kI * integral + kD * derivative;
        prevError = error;
        return output;
    }

    float PID::update(float sig, float pos){
        float error = sig - pos;   
        if (start == 0){
          prevError = error;
          start = 1;
        }
        integral += error;
        derivative = error - prevError;
        float output =  kP * error + kI * integral + kD * derivative;
        prevError = error;
        return output;
    }

    void PID::reset(){
      integral = 0;
      start = 0;
      prevError = 0;
      out = 0;
      derivative = 0;
    }

PID angularPID(1,0.1,12);
PID lateralPID(1,0,6);

void initMotion(chassis* initC, std::vector<float> angV, std::vector<float> latV){
   mainChassis = initC;
   angularPID.changeVals(angV[0],angV[1],angV[2]);
   lateralPID.changeVals(latV[0],latV[1],latV[2]);
}

std::vector<double> toPointStep(float sigX,float sigY, std::vector<double> pos){
  
  float angled = atan2(sigY - pos[1], sigX - pos[0]) * (180/M_PI);
  outA = angularPID.update((angled));
  dist = hypot(sigX-pos[0],sigY-pos[1]);
  outL = lateralPID.update(dist);
  return {outL, outA, angleWraper(angled), dist};
  
}

void toPoint(float tarX, float tarY, float exit){
  int count = 0;

  angularPID.reset();
  lateralPID.reset();
    do{

      std::vector<double> outs = toPointStep(tarX, tarY, odom::getPos());
      left_out = outs[0] - outs[1];
      right_out = outs[0] + outs[1];
      count = count + 1;
      mainChassis->leftMotors->move_velocity(left_out);
      mainChassis->rightMotors->move_velocity(right_out);
        
      pros::delay(10);
    }while(dist > exit);
    left_out = 0;
    right_out = 0;
    mainChassis->leftMotors->move_velocity(left_out);
    mainChassis->rightMotors->move_velocity(right_out);
}

std::vector<double> toAngleStep(float theta, std::vector<double> pos){
  
  float angled = theta - (pos[2]);
  outA = angularPID.update(angleWraper(angled));

  return {outA, angled};
  
}

void toAng(float tarT, float exit){

  angularPID.reset();
  lateralPID.reset();
  float err;
    do{

      std::vector<double> outs = toAngleStep(tarT, odom::getPos());
      left_out = -outs[0];
      right_out = outs[0];
      mainChassis->leftMotors->move_velocity(left_out);
      mainChassis->rightMotors->move_velocity(right_out);  

      pros::delay(10);
      err = outs[1];
    }while(err > exit);
    left_out = 0;
    right_out = 0;
    mainChassis->leftMotors->move_velocity(left_out);
    mainChassis->rightMotors->move_velocity(right_out);
}

std::vector<double> getOuts(){
  return {right_out,left_out};
} 

float pt_to_pt_distance (Point pt1,Point pt2){
    float distance = std::sqrt(std::pow((pt2.x - pt1.x), 2) + std::pow((pt2.y - pt1.y),2));
    return distance;
}

float pt_to_pt_distance (Point pt1,std::pair<float, float> pt2){
    float distance = std::sqrt(std::pow((pt2.first - pt1.x), 2) + std::pow((pt2.second - pt1.y),2));
    return distance;
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

void follow(std::vector<std::pair<float,float>> path, float exit, float lookDis){
  angularPID.reset();
  lateralPID.reset();
  float LFINDEX = 0;
  std::pair<float,float> last = path.back();
  Point curPOS = odom::getPoint();
  do{
      curPOS = odom::getPoint();
      std::vector<float> outs = pure_pursuit_step(path, curPOS, odom::getAng(),lookDis,LFINDEX);

      dist = std::hypot(last.first-curPOS.x,last.second-curPOS.y);
      outL = lateralPID.update(dist);
      left_out = outL - outs[3];
      right_out = outL + outs[3];
      LFINDEX = outs[2];

      mainChassis->leftMotors->move_velocity(left_out);
      mainChassis->rightMotors->move_velocity(right_out);
        
      pros::delay(10);
    }while(dist > exit);
    left_out = 0;
    right_out = 0;
    mainChassis->leftMotors->move_velocity(left_out);
    mainChassis->rightMotors->move_velocity(right_out);
}