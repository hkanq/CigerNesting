#include "geometry/collision.h"

#include "geometry/winding.h"
#include <algorithm>
#include <cmath>

namespace nest {
namespace {

double orientation(Vec2 a, Vec2 b, Vec2 c) {
    return cross(b - a, c - a);
}

bool onSegment(Vec2 a, Vec2 b, Vec2 p, double tolerance) {
    return std::abs(orientation(a, b, p)) <= tolerance &&
        p.x >= std::min(a.x, b.x) - tolerance && p.x <= std::max(a.x, b.x) + tolerance &&
        p.y >= std::min(a.y, b.y) - tolerance && p.y <= std::max(a.y, b.y) + tolerance;
}

AABB ringBounds(const Ring& ring) {
    AABB box;
    for (const auto& p : ring.points) {
        box.include(p);
    }
    return box;
}

} // namespace

bool segmentsIntersect(Vec2 a, Vec2 b, Vec2 c, Vec2 d, double tolerance) {
    const double o1 = orientation(a, b, c);
    const double o2 = orientation(a, b, d);
    const double o3 = orientation(c, d, a);
    const double o4 = orientation(c, d, b);

    if (((o1 > tolerance && o2 < -tolerance) || (o1 < -tolerance && o2 > tolerance)) &&
        ((o3 > tolerance && o4 < -tolerance) || (o3 < -tolerance && o4 > tolerance))) {
        return true;
    }

    return onSegment(a, b, c, tolerance) || onSegment(a, b, d, tolerance) ||
           onSegment(c, d, a, tolerance) || onSegment(c, d, b, tolerance);
}

bool ringsOverlap(const Ring& a, const Ring& b, double tolerance) {
    if (a.points.size() < 2 || b.points.size() < 2) {
        return false;
    }

    if (!ringBounds(a).expanded(tolerance).overlaps(ringBounds(b).expanded(tolerance))) {
        return false;
    }

    for (size_t i = 0; i + 1 < a.points.size(); ++i) {
        for (size_t j = 0; j + 1 < b.points.size(); ++j) {
            if (segmentsIntersect(a.points[i], a.points[i + 1], b.points[j], b.points[j + 1], tolerance)) {
                return true;
            }
        }
    }

    if (a.points.size() >= 3 && b.points.size() >= 3) {
        if (isPointInRing(a.points.front(), b.points) || isPointInRing(b.points.front(), a.points)) {
            return true;
        }
    }

    return false;
}

bool partsOverlap(const Part& a, const Pose& poseA, const Part& b, const Pose& poseB, double tolerance) {
    const auto ringsA = transformRings(a.rings, poseA);
    const auto ringsB = transformRings(b.rings, poseB);
    for (const auto& ra : ringsA) {
        if (ra.isHole) {
            continue;
        }
        for (const auto& rb : ringsB) {
            if (rb.isHole) {
                continue;
            }
            if (ringsOverlap(ra, rb, tolerance)) {
                return true;
            }
        }
    }
    return false;
}

double aabbOverlapArea(const AABB& a, const AABB& b) {
    if (!a.overlaps(b)) {
        return 0.0;
    }
    const double w = std::max(0.0, std::min(a.max.x, b.max.x) - std::max(a.min.x, b.min.x));
    const double h = std::max(0.0, std::min(a.max.y, b.max.y) - std::max(a.min.y, b.min.y));
    return w * h;
}

} // namespace nest
