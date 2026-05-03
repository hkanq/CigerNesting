#pragma once

#include <algorithm>
#include <cmath>

namespace nest {

constexpr double pi = 3.141592653589793238462643383279502884;
constexpr double twoPi = 2.0 * pi;

inline double degreesToRadians(double degrees) {
    return degrees * pi / 180.0;
}

inline double radiansToDegrees(double radians) {
    return radians * 180.0 / pi;
}

inline double clamp(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

inline bool nearlyZero(double value, double eps = 1e-9) {
    return std::abs(value) <= eps;
}

} // namespace nest
