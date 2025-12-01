#pragma once

#include <vector>

namespace gen{
enum degree{
    TWO
    THREE
    FOUR
    FIVE
}
std::vector<float> spline(enum degree deg, std::vector<std::pair<float, float>> pts, int samples);
}