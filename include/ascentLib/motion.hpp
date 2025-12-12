#pragma once

#include <vector>
#include "main.h"

class PID{
    public:
    float kP;
    float kI;
    float kD;
    float integral = 0;
    int start = 0;
    float prevError = 0;
    float out = 0;
    float derivative;
    PID(float inkP, float inkI, float inkD);
    void changeVals(float inkP, float inkI, float inkD);
    float update(float sig);
    float update(float sig, float pos);
    void reset();
    
};

std::vector<double> toPointStep(float sigX,float sigY, std::vector<float> pos);
void toPoint(float tarX, float tarY, float exit);
std::vector<double> getOuts();

std::vector<double> toPointStep(float sigX,float sigY, std::vector<float> pos);
void toPoint(float tarT, float exit);

struct chassis {
        pros::IMU* imu;
        pros::MotorGroup* leftMotors;
        pros::MotorGroup* rightMotors;
        pros::Rotation* horiz;
        pros::Rotation* vert;

        chassis(pros::IMU* val, pros::MotorGroup* val1, pros::MotorGroup* val2, pros::Rotation* val3, pros::Rotation* val4) : imu(val), leftMotors(val1), rightMotors(val1), horiz(val3), vert(val4) {}
    };

struct Point {
            float x;
            float y;

            Point(double x_val, double y_val) : x(x_val), y(y_val) {}
        };