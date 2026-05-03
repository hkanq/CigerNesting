#pragma once

#include <cmath>

namespace nest {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    constexpr Vec2() = default;
    constexpr Vec2(double px, double py) : x(px), y(py) {}

    constexpr Vec2 operator+(const Vec2& other) const { return {x + other.x, y + other.y}; }
    constexpr Vec2 operator-(const Vec2& other) const { return {x - other.x, y - other.y}; }
    constexpr Vec2 operator*(double scalar) const { return {x * scalar, y * scalar}; }
    constexpr Vec2 operator/(double scalar) const { return {x / scalar, y / scalar}; }

    Vec2& operator+=(const Vec2& other) { x += other.x; y += other.y; return *this; }
    Vec2& operator-=(const Vec2& other) { x -= other.x; y -= other.y; return *this; }
    Vec2& operator*=(double scalar) { x *= scalar; y *= scalar; return *this; }

    double lengthSquared() const { return x * x + y * y; }
    double length() const { return std::sqrt(lengthSquared()); }
};

inline constexpr Vec2 operator*(double scalar, const Vec2& value) {
    return value * scalar;
}

inline constexpr double dot(const Vec2& a, const Vec2& b) {
    return a.x * b.x + a.y * b.y;
}

inline constexpr double cross(const Vec2& a, const Vec2& b) {
    return a.x * b.y - a.y * b.x;
}

inline double distance(const Vec2& a, const Vec2& b) {
    return (a - b).length();
}

inline bool almostEqual(const Vec2& a, const Vec2& b, double eps = 1e-9) {
    return std::abs(a.x - b.x) <= eps && std::abs(a.y - b.y) <= eps;
}

} // namespace nest
