#pragma once

#include "core/vec2.h"
#include <cmath>

namespace nest {

struct Mat2 {
    double m00 = 1.0;
    double m01 = 0.0;
    double m10 = 0.0;
    double m11 = 1.0;

    constexpr Mat2() = default;
    constexpr Mat2(double a, double b, double c, double d) : m00(a), m01(b), m10(c), m11(d) {}

    static constexpr Mat2 identity() { return {}; }
    static constexpr Mat2 scale(double sx, double sy) { return {sx, 0.0, 0.0, sy}; }

    static Mat2 rotation(double radians) {
        const double c = std::cos(radians);
        const double s = std::sin(radians);
        return {c, -s, s, c};
    }

    constexpr Vec2 apply(const Vec2& p) const {
        return {m00 * p.x + m01 * p.y, m10 * p.x + m11 * p.y};
    }

    constexpr Mat2 operator*(const Mat2& rhs) const {
        return {
            m00 * rhs.m00 + m01 * rhs.m10,
            m00 * rhs.m01 + m01 * rhs.m11,
            m10 * rhs.m00 + m11 * rhs.m10,
            m10 * rhs.m01 + m11 * rhs.m11
        };
    }
};

} // namespace nest
