#pragma once

#include "core/aabb.h"
#include "core/polygon.h"
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace nest {

inline double ringSignedArea(const Ring& ring) {
    if (ring.points.size() < 3) {
        return 0.0;
    }
    double sum = 0.0;
    for (size_t i = 0; i < ring.points.size(); ++i) {
        const Vec2& a = ring.points[i];
        const Vec2& b = ring.points[(i + 1) % ring.points.size()];
        sum += a.x * b.y - b.x * a.y;
    }
    return sum * 0.5;
}

class Part {
public:
    std::wstring name;
    std::vector<Ring> rings;
    Pose pose;
    AABB localBounds;
    double area = 0.0;

    void updateDerivedGeometry() {
        localBounds = {};
        area = 0.0;
        for (auto& ring : rings) {
            for (const auto& point : ring.points) {
                localBounds.include(point);
            }
            const double signedArea = ringSignedArea(ring);
            ring.winding = signedArea >= 0.0 ? 1 : -1;
            area += ring.isHole ? -std::abs(signedArea) : std::abs(signedArea);
        }
        if (area < 0.0) {
            area = 0.0;
        }
    }

    bool empty() const { return rings.empty(); }
};

inline std::vector<Ring> transformRings(const std::vector<Ring>& rings, const Pose& pose) {
    const Transform transform = pose.toTransform();
    std::vector<Ring> out;
    out.reserve(rings.size());
    for (const auto& ring : rings) {
        Ring transformed = ring;
        for (auto& point : transformed.points) {
            point = transform.apply(point);
        }
        out.push_back(std::move(transformed));
    }
    return out;
}

inline AABB transformedBounds(const Part& part, const Pose& pose) {
    const Transform transform = pose.toTransform();
    AABB box;
    for (const auto& ring : part.rings) {
        for (const auto& point : ring.points) {
            box.include(transform.apply(point));
        }
    }
    return box;
}

} // namespace nest
