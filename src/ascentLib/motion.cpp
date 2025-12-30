#include "ascentLib/motion.hpp"
#include <cmath>


float right_out = 0;
float left_out = 0;
float dist;



int sgn(float val){
  if (val > 0){
    return 1;
  }else if(val < 0){
    return -1;
  }else{
    return 0;
  }
}

double constrainAngle(double x){
    x = fmod(x + 180,360);
    if (x < 0)
        x += 360;
    return x - 180;
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
        output =  kP * error + kI * integral + kD * derivative;
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
        output =  kP * error + kI * integral + kD * derivative;
        prevError = error;
        return output;
    }

    void PID::reset(){
      integral = 0;
      start = 0;
      prevError = 0;
      out = 0;
      derivative = 0;
      output = 0;
    }

PID angularPID(0.1,0,6);
PID lateralPID(3,0,6);

chassis* motionChassis;

void initMotion(chassis* initC, std::vector<float> angV, std::vector<float> latV){
   motionChassis = initC;
   angularPID.changeVals(angV[0],angV[1],angV[2]);
   lateralPID.changeVals(latV[0],latV[1],latV[2]);
}

std::vector<double> toPointStep(float sigX,float sigY, std::vector<double> pos, float angled){
  
  
  float wrapped = fmod(((-pos[2])-180),360)+180;
  outA = angularPID.update(fmod(((angled-wrapped)-180),360)+180);
  dist = hypot(sigX-pos[0],sigY-pos[1]);
  outL = lateralPID.update(dist);
  return {outL, outA, wrapped, dist};
  
}

void toPoint(float tarX, float tarY, float exit, moveParams params){
  int count = 0;

  angularPID.reset();
  lateralPID.reset();
  std::vector<double> pos = odom::getPos();
  float dist;
  //float angled = atan2(tarY - pos[1], tarX - pos[0]) * (180/M_PI);
  float targetAngle = atan2(tarX - pos[0], tarY - pos[1]) * (180/M_PI);
  float dx = tarX - pos[0];
  float dy = tarY - pos[1];
  float rawDist;
    do{
      if (!params.steady) targetAngle = atan2(tarX - pos[0], tarY - pos[1]) * (180/M_PI);
      std::vector<double> pos = odom::getPos();
      if(params.reversed) pos[2] += 180;
      
      float dx = tarX - pos[0];
      float dy = tarY - pos[1];
      rawDist = std::hypot(dx, dy); 
 
      float angleError = constrainAngle(targetAngle - pos[2]);
    

      float angleRad = odom::getAng() * (M_PI / 180.0);
      float forwardX = sin(angleRad);
      float forwardY = cos(angleRad);
      dist = (dx * forwardX + dy * forwardY); 

      if(params.straight || rawDist < params.settleDist) angleError = 0;

      if(params.cosScalling) dist = dist * std::cos((angleError * M_PI) / 180.0);
      
      if(params.cosScalling && std::cos((angleError * M_PI) / 180.0) < 0) dist = 0;
      

      outA = angularPID.update(angleError);
      outL = lateralPID.update(dist);      

      left_out = outL - outA;
      right_out = outL + outA;

      float max = fmax(std::abs(left_out), std::abs(right_out));
      float MAXMOTOR = 200;
      if (max > MAXMOTOR){
        left_out = (left_out / max) * MAXMOTOR;
        right_out = (right_out / max) * MAXMOTOR;
      }

      count = count + 1;

      motionChassis->leftMotors->move_velocity(left_out);
      motionChassis->rightMotors->move_velocity(right_out);
      pros::lcd::print(0, "L: %f A: %f", dist, angleError);
      //pros::lcd::print(2, "X: %f Y: %f", pos[0], pos[1]); 
      pros::delay(10);
    }while(std::abs(rawDist) > exit);
    left_out = 0;
    right_out = 0;
    motionChassis->leftMotors->move_velocity(left_out);
    motionChassis->rightMotors->move_velocity(right_out);
}


void toPose(float tarX, float tarY, float tarT, float exit, moveParams params){
  int count = 0;

  angularPID.reset();
  lateralPID.reset();
  std::vector<double> pos = odom::getPos();
  float dist;
  //float angled = atan2(tarY - pos[1], tarX - pos[0]) * (180/M_PI);
  float targetAngle = atan2(tarX - pos[0], tarY - pos[1]) * (180/M_PI);
  float dx = tarX - pos[0];
  float dy = tarY - pos[1];
  float rawDist = hypot(dx, dy);
  float carrDist;
  float angleError;
    do{
      std::vector<double> pos = odom::getPos();

      Point carr(tarX - rawDist * cos(tarT) * params.dlead,
			         tarY - rawDist * sin(tarT) * params.dlead);
      targetAngle = atan2(carr.x - pos[0], carr.y - pos[1]) * (180/M_PI);

      if(params.reversed) pos[2] += 180;
      
      float Cdx = carr.x - pos[0];
      float Cdy = carr.y - pos[1];
      carrDist = std::hypot(Cdx, Cdy); 
      
      float dx = tarX - pos[0];
      float dy = tarY - pos[1];
      rawDist = std::hypot(dx, dy); 
      if(params.settleDist > 0 && rawDist < params.settleDist){
        targetAngle = tarT;
      }
      angleError = constrainAngle(targetAngle - pos[2]);
    

      float angleRad = odom::getAng() * (M_PI / 180.0);
      float forwardX = sin(angleRad);
      float forwardY = cos(angleRad);
      dist = (Cdx * forwardX + Cdy * forwardY); 

      

      outA = angularPID.update(angleError);

      outL = lateralPID.update(dist);

      left_out = outL - outA;
      right_out = outL + outA;
      count = count + 1;
      motionChassis->leftMotors->move_velocity(left_out);
      motionChassis->rightMotors->move_velocity(right_out);
      pros::lcd::print(0, "L: %f A: %f", dist, angleError);
      //pros::lcd::print(0, "X: %f Y: %f", carr.x, carr.y); 
      pros::delay(10);
    }while(std::abs(rawDist) > exit || std::abs(angleError) > 5);
    left_out = 0;
    right_out = 0;
    motionChassis->leftMotors->move_velocity(left_out);
    motionChassis->rightMotors->move_velocity(right_out);
}


void toAng(float tarT, float exit){

  angularPID.reset();
  lateralPID.reset();
  float err;
  int settleCount = 0;
    do{
      float angled = constrainAngle(tarT - odom::getAng());
      outA = angularPID.update(constrainAngle(angled));
      left_out = -outA;
      right_out = outA;
      motionChassis->rightMotors->move_velocity(right_out);
      motionChassis->leftMotors->move_velocity(left_out);

      pros::lcd::print(0, "err: %f", angled);

      pros::delay(10);
      err = angled;
      if (std::abs(err) < exit) {
            settleCount++;
        } else {
            settleCount = 0;
        }

        if (settleCount > 10) break;
    }while(true);
    left_out = 0;
    right_out = 0;
    motionChassis->leftMotors->move_velocity(left_out);
    motionChassis->rightMotors->move_velocity(right_out);
    motionChassis->leftMotors->brake();
    motionChassis->rightMotors->brake();
}

void turnToPoint(float tarX, float tarY, float exit){

  angularPID.reset();
  lateralPID.reset();
  float err;
  std::vector<double> pos = odom::getPos();
  float targetAngle = atan2(tarX - pos[0], tarY - pos[1]) * (180/M_PI);
    int settleCount = 0;
    do{

      float currentHeading = odom::getAng();
      float angleError = targetAngle - currentHeading;

      angleError = constrainAngle(angleError);

      outA = angularPID.update(angleError);
      left_out = -outA;
      right_out = outA;
      motionChassis->rightMotors->move_velocity(right_out);
      motionChassis->leftMotors->move_velocity(left_out);

      pros::lcd::print(0, "X: %f Y: %f error: %f", pos[0], pos[1], angleError);

      pros::delay(10);
      err = angleError;
      if (std::abs(err) < exit) {
            settleCount++;
        } else {
            settleCount = 0;
        }

        if (settleCount > 10) break;
    }while(true);
    //}while(1==2);
    left_out = 0;
    right_out = 0;
    motionChassis->leftMotors->move_velocity(left_out);
    motionChassis->rightMotors->move_velocity(right_out);
}

void toDistance(float dist, float exit){

  angularPID.reset();
  lateralPID.reset();
  float err;
  float start = motionChassis->leftMotors->get_position();
  float target = start + dist;
  int settleCount = 0;
  float current;
  do{

      current = motionChassis->leftMotors->get_position();
      err = target - current;

      outL = lateralPID.update(err);
      left_out = outL;
      right_out = outL;
      motionChassis->rightMotors->move_velocity(right_out);
      motionChassis->leftMotors->move_velocity(left_out);

      pros::lcd::print(0, "error: %f at: %f", err, current);

      pros::delay(10);
      
      if (std::abs(err) < exit) {
            settleCount++;
        } else {
            settleCount = 0;
        }

        if (settleCount > 10) break;
    }while(true);
    left_out = 0;
    right_out = 0;
    motionChassis->leftMotors->move_velocity(left_out);
    motionChassis->rightMotors->move_velocity(right_out);
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

      if (((minX <= sol_pt1.x && sol_pt1.x <= maxX) && (minY <= sol_pt1.y && sol_pt1.y <= maxY)) || ((minX <= sol_pt2.x && sol_pt2.x <= maxX) && (minY <= sol_pt2.y && sol_pt2.y <= maxY))){
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

      motionChassis->leftMotors->move_velocity(left_out);
      motionChassis->rightMotors->move_velocity(right_out);
        
      pros::delay(10);
    }while(dist > exit);
    left_out = 0;
    right_out = 0;
    motionChassis->leftMotors->move_velocity(left_out);
    motionChassis->rightMotors->move_velocity(right_out);
}