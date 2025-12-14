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