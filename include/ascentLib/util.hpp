#pragma once
#include <limits>

struct Point {
            float x;
            float y;
            float theta = std::numeric_limits<float>::quiet_NaN();

        // Constructor
            Point(float x, float y, float theta = std::numeric_limits<float>::quiet_NaN())
                : x(x),
                y(y),
                theta(theta) {}

        };

struct chassis{
        pros::IMU* imu;
        pros::MotorGroup* leftMotors;
        pros::MotorGroup* rightMotors;
        pros::Rotation* horiz;
        pros::Rotation* vert;

        chassis(pros::IMU* val, pros::MotorGroup* val1, pros::MotorGroup* val2, pros::Rotation* val3, pros::Rotation* val4) : imu(val), leftMotors(val1), rightMotors(val2), horiz(val3), vert(val4) {}
    };

chassis* mainChassis;