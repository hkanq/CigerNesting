#include "geometry/polygon_utils.h"

#include "geometry/winding.h"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace nest {

AABB boundsOfRing(const Ring& ring) {
    AABB box;
    for (const auto& point : ring.points) {
        box.include(point);
    }
    return box;
}

Ring transformedRing(const Ring& ring, const Transform& transform) {
    Ring out = ring;
    for (auto& point : out.points) {
        point = transform.apply(point);
    }
    return out;
}

std::vector<Ring> transformedRings(const std::vector<Ring>& rings, const Transform& transform) {
    std::vector<Ring> out;
    out.reserve(rings.size());
    for (const auto& ring : rings) {
        out.push_back(transformedRing(ring, transform));
    }
    return out;
}

void classifyHoleRings(std::vector<Ring>& rings) {
    if (rings.empty()) {
        return;
    }

    std::vector<size_t> order(rings.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return std::abs(signedArea(rings[a].points)) > std::abs(signedArea(rings[b].points));
    });

    for (auto& ring : rings) {
        ring.isHole = false;
        ring.winding = windingDirection(ring.points);
    }

    for (size_t idx = 1; idx < order.size(); ++idx) {
        Ring& candidate = rings[order[idx]];
        if (candidate.points.empty()) {
            continue;
        }
        for (size_t outerIdx = 0; outerIdx < idx; ++outerIdx) {
            const Ring& outer = rings[order[outerIdx]];
            if (!outer.points.empty() && isPointInRing(candidate.points.front(), outer.points)) {
                candidate.isHole = true;
                break;
            }
        }
    }
}

double polygonArea(const std::vector<Ring>& rings) {
    double area = 0.0;
    for (const auto& ring : rings) {
        const double a = std::abs(signedArea(ring.points));
        area += ring.isHole ? -a : a;
    }
    return std::max(0.0, area);
}

} // namespace nest
