#include "main.h" // IWYU pragma: keep
#include "ascentLib/odom.hpp"
#include "ascentLib/motion.hpp"
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
}

std::vector<double> getOuts(){
  return {right_out,left_out};
} 
