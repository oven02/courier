#pragma once

#include <vector>
#include "main.h"
#include "ascentLib/util.hpp"

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
    float output;
    PID(float inkP, float inkI, float inkD);
    void changeVals(float inkP, float inkI, float inkD);
    float update(float sig);
    float update(float sig, float pos);
    void reset();
    
};



void initMotion(chassis* initC, std::vector<float> angV = {0.01,0,12}, std::vector<float> latV = {3,0,6});

std::vector<double> toPointStep(float sigX,float sigY, std::vector<float> pos);
void toPoint(float tarX, float tarY, float exit, bool reversed = false);

std::vector<double> toAngleStep(float sigX,float sigY, std::vector<float> pos);
void toAng(float tarT, float exit);

void turnToPoint(float sigX, float sigY, float exit);

std::vector<double> getOuts();

std::vector<double> pure_pursuit_step(float sigX,float sigY, std::vector<float> pos);
void follow(float tarT, float exit);


