#pragma once

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



void odomCalc();
void updateDeltas();
void updatePrev();
void odomDrive(void* param);
void odomX(void* param);
void odomY(void* param);
void odomXY(void* param);
bool init_odom(enum odom::config con, TaskParams params);
bool init_odom(enum odom::config con, float sV_in, int imu_port, int tracking_port, float wheelDiameter);
bool init_odom(enum odom::config con, float sV_in, float sS_in, int imu_port, int horiz_port, int vert_port, float YwheelDiameter, float XwheelDiameter);
std::vector<double> getPos();
}