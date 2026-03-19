#pragma once

#include <vector>
#include <utility>
#include <cmath>

namespace gen{
enum degree{
    TWO,
    THREE,
    FOUR,
    FIVE
};
std::vector<std::pair<float, float>> spline(enum degree deg, std::vector<std::pair<float, float>> pts, int samples);
}