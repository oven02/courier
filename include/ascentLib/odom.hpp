#pragma once

#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "ascentLib/util.hpp"
#include <vector>

namespace odom {
enum config {
    DRIVE,
    XTRACK,
    YTRACK,
    XYTRACK
};


struct TaskParams {
        float sV_in;
        float sS_in;
        int imu_port;
        int vert_port;
        int horiz_port;
        float YwheelDiameter;
        float XwheelDiameter;
        float DriveRatio;
    };

struct initParams {
        float sV_in;
        float sS_in;
        float YwheelDiameter;
        float XwheelDiameter;
        float DriveRatio;
        pros::IMU* imu;
        pros::MotorGroup* driveMotor;
        pros::Rotation* horiz;

        initParams(pros::IMU* val, pros::MotorGroup* val1, pros::Rotation* val2) : imu(val), driveMotor(val1), horiz(val2) {}
    };

struct odomParams {
        float sV_in;
        float sS_in;
        float YwheelDiameter;
        float XwheelDiameter;
        float DriveRatio;
        pros::IMU* imu;
        pros::Rotation* vert;
        pros::Rotation* horiz;

        odomParams(pros::IMU* val, pros::Rotation* val1, pros::Rotation* val2) : imu(val), vert(val1), horiz(val2) {}
    };

void odomCalc();
void updateDeltas();
void updatePrev();
void odomDrive(void* param);
void odomX(void* param);
void odomY(void* param);
void odomXY(void* param);
bool init_odom(enum odom::config con, initParams params);
bool init_odom(enum odom::config con, odomParams params);
bool init_odom(enum odom::config con, float sV_in, int imu_port, int tracking_port, float wheelDiameter);
bool init_odom(enum odom::config con, float sV_in, float sS_in, int imu_port, int horiz_port, int vert_port, float YwheelDiameter, float XwheelDiameter);
std::vector<double> getPos();
std::vector<double> getVals();
Point getPoint();
float getAng();

}