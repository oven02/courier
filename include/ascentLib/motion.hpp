#pragma once

#include <vector>
#include <main.h>

class PID{
    public:
    PID(float inkP, float inkI, float inkD);
    void changeVals(float inkP, float inkI, float inkD);
    float update(float sig);
    float update(float sig, float pos);
    void reset();
    
}

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

        initParams(pros::IMU* val, pros::MotorGroup* val1, pros::MotorGroup* val2, pros::Rotation* val3, pros::Rotation* val4) : imu(val), leftMotors(val1), rightMotors(val1), horiz(val2), vert(val2) {}
    };

