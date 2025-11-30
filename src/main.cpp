#include "main.h" // IWYU pragma: keep
#include "odom.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include <iostream> 
#include <cstdlib> // For integer abs()
#include <cmath>
#include <utility>
#include <algorithm> 

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}


pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({1});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({-3});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
pros::IMU imu (5);

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

namespace control{
float outA;
float outL; 
class PID{
  public:
  float kP;
  float kI;
  float kD;
//        self.sig = 1
  float integral = 0;
  int start = 0;
  float prevError = 0;
  float out = 0;
  float derivative;
    PID(float inkP, float inkI, float inkD){
        kP = inkP;
        kI = inkI;
        kD = inkD;
    }

    float update(float sig){
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

    float update(float sig, float pos){
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
};

PID angularPID(1,0.1,12);
PID lateralPID(1,0,6);

// std::vector<double> get_vals(float sigX,float sigY, int dir){
  
//   float angled = atan2(sigY - odom::yPos, sigX - odom::xPos) * (180/M_PI);
//   outA = angularPID.update(angleWraper(angled - imu.get_heading()));
//   float dist = hypot(sigX-odom::xPos,sigY-odom::yPos);
//   outL = lateralPID.update(dist*dir);
//   return {outL, outA, angleWraper(angled - imu.get_heading()), dist};
  
// }

std::vector<double> get_vals(float sigX,float sigY){
  
//   float angled = atan2(sigY - odom::yPos, sigX - odom::xPos) * (180/M_PI);
//   outA = angularPID.update(angleWraper(angled - imu.get_heading()));
//   float dist = hypot(sigX-odom::xPos,sigY-odom::yPos);
//   outL = lateralPID.update(dist);
//   return {outL, outA, angleWraper(angled - imu.get_heading()), dist};
return {0,0,0,0};
  
}

void control(float tarX, float tarY, float tarTheta){
  int count = 0;
    do{

      std::vector<double> outs = get_vals(tarX, tarY);
      float left_out = outs[0] - outs[1];
      float right_out = outs[0] + outs[1];
      count = count + 1;
      pros::lcd::print(0, "%f %f", outs[2], outs[3]); 
                    
      // float drive = lateralPID.update(50, right_mg.get_position() * (4 * M_PI) * (0.6) / 360);
      // float left_out = drive;
      // float right_out =  drive;

      right_mg.move_velocity(right_out);
      left_mg.move_velocity(left_out);  
      // if (dist_err < 0.5){
      //   break;
      // }    
      pros::delay(10);
    }while(1==1);
    pros::lcd::print(0, "All done");
    right_mg.move_velocity(0);
    left_mg.move_velocity(0); 
}

}


  


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

  
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

  imu.reset(true);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

  odom::init_odom(odom::DRIVE, 7, 5, 1, 4);
	
	
  //control::control(10,20,0);

}