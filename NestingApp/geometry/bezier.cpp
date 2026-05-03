#include "geometry/bezier.h"

#include <cmath>

namespace nest {
namespace {

double pointLineDistance(const Vec2& p, const Vec2& a, const Vec2& b) {
    const Vec2 ab = b - a;
    const double len2 = ab.lengthSquared();
    if (len2 <= 1e-12) {
        return distance(p, a);
    }
    return std::abs(cross(ab, p - a)) / std::sqrt(len2);
}

void flattenQuadraticRecursive(const Vec2& p0, const Vec2& p1, const Vec2& p2, double tolerance, int depth, std::vector<Vec2>& out) {
    if (depth > 14 || pointLineDistance(p1, p0, p2) <= tolerance) {
        out.push_back(p2);
        return;
    }

    const Vec2 p01 = (p0 + p1) * 0.5;
    const Vec2 p12 = (p1 + p2) * 0.5;
    const Vec2 p012 = (p01 + p12) * 0.5;
    flattenQuadraticRecursive(p0, p01, p012, tolerance, depth + 1, out);
    flattenQuadraticRecursive(p012, p12, p2, tolerance, depth + 1, out);
}

void flattenCubicRecursive(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, double tolerance, int depth, std::vector<Vec2>& out) {
    const double d1 = pointLineDistance(p1, p0, p3);
    const double d2 = pointLineDistance(p2, p0, p3);
    if (depth > 16 || std::max(d1, d2) <= tolerance) {
        out.push_back(p3);
        return;
    }

    const Vec2 p01 = (p0 + p1) * 0.5;
    const Vec2 p12 = (p1 + p2) * 0.5;
    const Vec2 p23 = (p2 + p3) * 0.5;
    const Vec2 p012 = (p01 + p12) * 0.5;
    const Vec2 p123 = (p12 + p23) * 0.5;
    const Vec2 p0123 = (p012 + p123) * 0.5;

    flattenCubicRecursive(p0, p01, p012, p0123, tolerance, depth + 1, out);
    flattenCubicRecursive(p0123, p123, p23, p3, tolerance, depth + 1, out);
}

} // namespace

void flattenQuadraticBezier(const Vec2& p0, const Vec2& p1, const Vec2& p2, double tolerance, std::vector<Vec2>& out) {
    flattenQuadraticRecursive(p0, p1, p2, std::max(1e-4, tolerance), 0, out);
}

void flattenCubicBezier(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, double tolerance, std::vector<Vec2>& out) {
    flattenCubicRecursive(p0, p1, p2, p3, std::max(1e-4, tolerance), 0, out);
}

} // namespace nest
