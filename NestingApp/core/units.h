#pragma once

namespace nest {

constexpr double hpglUnitsPerMillimeter = 40.0;
constexpr double defaultCurveFlattenTolerance = 0.35;

inline double hpglToMillimeters(double value) {
    return value / hpglUnitsPerMillimeter;
}

} // namespace nest
