#include "geometry/winding.h"

#include <cmath>

namespace nest {

double signedArea(const std::vector<Vec2>& points) {
    if (points.size() < 3) {
        return 0.0;
    }
    double sum = 0.0;
    for (size_t i = 0; i < points.size(); ++i) {
        const Vec2& a = points[i];
        const Vec2& b = points[(i + 1) % points.size()];
        sum += a.x * b.y - b.x * a.y;
    }
    return sum * 0.5;
}

int windingDirection(const std::vector<Vec2>& points) {
    const double area = signedArea(points);
    if (std::abs(area) < 1e-9) {
        return 0;
    }
    return area > 0.0 ? 1 : -1;
}

bool isPointInRing(const Vec2& p, const std::vector<Vec2>& ring) {
    bool inside = false;
    if (ring.size() < 3) {
        return false;
    }

    for (size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
        const Vec2& pi = ring[i];
        const Vec2& pj = ring[j];
        const bool intersects = ((pi.y > p.y) != (pj.y > p.y)) &&
            (p.x < (pj.x - pi.x) * (p.y - pi.y) / ((pj.y - pi.y) + 1e-20) + pi.x);
        if (intersects) {
            inside = !inside;
        }
    }
    return inside;
}

} // namespace nest
