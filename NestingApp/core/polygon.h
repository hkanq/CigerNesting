#pragma once

#include "core/aabb.h"
#include "core/transform.h"
#include "core/vec2.h"
#include <vector>

namespace nest {

struct Ring {
    std::vector<Vec2> points;
    bool isHole = false;
    int winding = 0;

    bool closed(double eps = 1e-9) const {
        return points.size() > 2 && almostEqual(points.front(), points.back(), eps);
    }
};

struct Polygon {
    std::vector<Ring> rings;

    AABB bounds() const {
        AABB box;
        for (const auto& ring : rings) {
            for (const auto& point : ring.points) {
                box.include(point);
            }
        }
        return box;
    }
};

struct Pose {
    double x = 0.0;
    double y = 0.0;
    double angleRadians = 0.0;
    bool mirrored = false;

    Transform toTransform() const {
        const Transform mirror = mirrored ? Transform::scaled(-1.0, 1.0) : Transform::identity();
        return Transform::translated(x, y) * Transform::rotated(angleRadians) * mirror;
    }
};

} // namespace nest
