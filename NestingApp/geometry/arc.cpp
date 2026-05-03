#include "geometry/arc.h"

#include "core/math_utils.h"
#include <algorithm>
#include <cmath>

namespace nest {

std::vector<Vec2> flattenCircularArc(Vec2 center, double radiusX, double radiusY, double startRadians, double sweepRadians, double tolerance) {
    const double radius = std::max(radiusX, radiusY);
    const double safeTolerance = std::max(0.01, tolerance);
    const double maxStep = 2.0 * std::acos(std::max(-1.0, std::min(1.0, 1.0 - safeTolerance / std::max(1.0, radius))));
    const int steps = std::max(8, static_cast<int>(std::ceil(std::abs(sweepRadians) / std::max(0.05, maxStep))));

    std::vector<Vec2> points;
    points.reserve(static_cast<size_t>(steps) + 1);
    for (int i = 0; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const double a = startRadians + sweepRadians * t;
        points.push_back({center.x + std::cos(a) * radiusX, center.y + std::sin(a) * radiusY});
    }
    return points;
}

} // namespace nest
