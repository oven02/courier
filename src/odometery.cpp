#include <vector>
#include <cmath>
#include "main.h" // IWYU pragma: keep
#include "odom.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

namespace odom{

float prevAngle = 0;
float prevHoriz = 0;
float prevVert = 0;

float thetaDelta = 0;
float vertDelta = 0;
float horizDelta = 0;

float theta;
float vert;
float horiz;

float xPos = 0;
float yPos = 0;
float sV;
float sS;

void updateDeltas(){
  thetaDelta = theta - prevAngle;
  vertDelta = vert - prevVert;
  horizDelta = horiz - prevHoriz;
}

void updatePrev(){
  prevAngle = theta;
  prevVert = vert;
  prevHoriz = horiz;
}

void odomCalc(){
  //  thetaDelta Will need to be in radians
  float deltaX;
  float deltaY;
  float rot;
  std::vector<float> DeltaCoords;

  //float angleDelta = (leftDelta - rightDelta) / (sL + sR); // if we lacked a IMU we could use this

  float angleDelta = thetaDelta;

  if (std::abs(angleDelta) > 0.0174533){

    DeltaCoords = {2*std::sin(angleDelta/2)*((horizDelta/angleDelta) + sS), 2*std::sin(angleDelta/2)*((vertDelta/angleDelta) + sV)};

  }else{
    DeltaCoords = {horizDelta, vertDelta}; 
  }
  
  rot = prevAngle + (angleDelta/2);
  //rot = rot*(180/M_PI);
  deltaX = DeltaCoords[0]*std::cos(rot) - DeltaCoords[1]*std::sin(rot); 
  deltaY = DeltaCoords[0]*std::sin(rot) + DeltaCoords[1]*std::cos(rot);

  xPos = xPos + deltaX; 
  yPos = yPos + deltaY;
}



void odomDrive(void* param){
TaskParams* params = static_cast<TaskParams*>(param);
// sV_in, float sS_in, int imu_port, int tracking_port

sS = params->sS_in;
sV = params->sV_in;
float diameter = params->YwheelDiameter;
float driveRatio = params->DriveRatio;

pros::IMU imu (params->imu_port);
pros::Motor vert_m(params->vert_port);


int go = 1;
while (go==1){
  horiz = 0;
  vert = vert_m.get_position() * (diameter * M_PI) * driveRatio / 360;// I used 4 inch wheels, so the 4 would be changed to what every size wheels And the 3/5 is the gear ratio
  theta = imu.get_heading() * (M_PI/180); // gets the inertial and converts to radians
  updateDeltas();
  odomCalc();
  updatePrev();

  pros::delay(10);
}
}

void odomY(void* param){
TaskParams* params = static_cast<TaskParams*>(param);
// sV_in, float sS_in, int imu_port, int tracking_port

sS = params->sS_in;
sV = params->sV_in;
float diameter = params->YwheelDiameter;

pros::IMU imu (params->imu_port);
pros::Rotation vert_m(params->vert_port);


int go = 1;
while (go==1){
  horiz = 0; 
  vert = (vert_m.get_position() * (diameter * M_PI)/ 360) / 100;// I used 4 inch wheels, so the 4 would be changed to what every size wheels And the 3/5 is the gear ratio
  theta = imu.get_heading() * (M_PI/180); // gets the inertial and converts to radians
  updateDeltas();
  odomCalc();
  updatePrev();

  pros::delay(10);
}
}

void odomX(void* param){
TaskParams* params = static_cast<TaskParams*>(param);
// sV_in, float sS_in, int imu_port, int tracking_port

sS = params->sS_in;
sV = params->sV_in;
float Xdiameter = params->XwheelDiameter;
float Ydiameter = params->YwheelDiameter;
float driveRatio = params->DriveRatio;


pros::Motor vert_m(params->vert_port);
pros::IMU imu (params->imu_port);
pros::Rotation horiz_m(params->horiz_port);


int go = 1;
while (go==1){
  horiz = (horiz_m.get_position() * (Xdiameter * M_PI)/ 360)/100; //This is temporary becuase I dont feel like seting up a traking wheel, and dont have a rotation sensor
  vert = (vert_m.get_position() * (Ydiameter * M_PI) * driveRatio / 360);// I used 4 inch wheels, so the 4 would be changed to what every size wheels And the 3/5 is the gear ratio
  theta = imu.get_heading() * (M_PI/180); // gets the inertial and converts to radians
  updateDeltas();
  odomCalc();
  updatePrev();

  pros::delay(10);
}
}

void odomXY(void* param){
TaskParams* params = static_cast<TaskParams*>(param);
// sV_in, float sS_in, int imu_port, int tracking_port

sS = params->sS_in;
sV = params->sV_in;
float YwheelDiameter = params->YwheelDiameter;
float XwheelDiameter = params->XwheelDiameter;

pros::IMU imu (params->imu_port);
pros::Rotation horiz_m(params->horiz_port);
pros::Rotation vert_m(params->vert_port);


int go = 1;
while (go==1){
  horiz = (horiz_m.get_position() * (XwheelDiameter * M_PI) / 360) / 100;
  vert = (vert_m.get_position() * (YwheelDiameter * M_PI) / 360) / 100;// I used 4 inch wheels, so the 4 would be changed to what every size wheels And the 3/5 is the gear ratio
  theta = imu.get_heading() * (M_PI/180); // gets the inertial and converts to radians
  updateDeltas();
  odomCalc();
  updatePrev();

  pros::delay(10);
}
}


//functions to initiallize the odom system

bool init_odom(enum odom::config con, float sV_in, int imu_port, int tracking_port, float wheelDiameter){
  //code to setup encoders and imu
  TaskParams inputParams;
    inputParams.sV_in = sV_in;
    inputParams.sS_in = 0;
    inputParams.imu_port = imu_port;
    inputParams.vert_port = tracking_port;
    inputParams.YwheelDiameter = wheelDiameter;

  if (con == odom::DRIVE){
    //setup for drive only
    pros::Task odo(odomDrive, &inputParams);
  }else if (con == odom::YTRACK){
    //setup for ytrack only
    pros::Task odo(odomY, &inputParams);
  }
  return true;
}

bool init_odom(enum odom::config con, float sV_in, float sS_in, int imu_port, int horiz_port, int vert_port, float YwheelDiameter, float XwheelDiameter){
  //code to setup encoders and imu
  TaskParams inputParams;
    inputParams.sV_in = sV_in;
    inputParams.sS_in = sS_in;
    inputParams.imu_port = imu_port;
    inputParams.vert_port = vert_port;
    inputParams.horiz_port = horiz_port;
    inputParams.YwheelDiameter = YwheelDiameter;
    inputParams.XwheelDiameter = XwheelDiameter;

  if (con == odom::YTRACK){
    //setup for xtrack only
    pros::Task odo(odomY, &inputParams);
  }else if (con == odom::XYTRACK){
    //setup for xytrack
    pros::Task odo(odomXY, &inputParams);
  }
  return true;
}

bool init_odom(enum odom::config con, TaskParams params){
  if (con == odom::DRIVE){
    //setup for drive only
    pros::Task odo(odomDrive, &params);
  }else if (con == odom::XTRACK){
    //setup for xtrack only
    pros::Task odo(odomX, &params);
  }else if (con == odom::YTRACK){
    //setup for ytrack only
    pros::Task odo(odomY, &params);
  }else if (con == odom::XYTRACK){
    //setup for xytrack
    pros::Task odo(odomXY, &params);
  }
  return true;

}

std::vector<double> getPos(){
  return {xPos, yPos, theta * (180/M_PI)};
}




} //Namespace odom