#include "engine/layout_score_components.h"

#include "geometry/collision.h"
#include "geometry/transformed_shape.h"
#include <algorithm>
#include <cmath>

namespace nest {
namespace {

bool documentHasHoles(const Document& document) {
    for (const Part& part : document.parts) {
        for (const Ring& ring : part.rings) {
            if (ring.isHole && ring.points.size() >= 3) {
                return true;
            }
        }
    }
    return false;
}

double ringArea(const std::vector<Vec2>& points) {
    if (points.size() < 3) {
        return 0.0;
    }
    const bool closed = almostEqual(points.front(), points.back(), 1e-9);
    const size_t count = closed ? points.size() - 1 : points.size();
    if (count < 3) {
        return 0.0;
    }
    double sum = 0.0;
    for (size_t i = 0; i < count; ++i) {
        const Vec2& a = points[i];
        const Vec2& b = points[(i + 1) % count];
        sum += a.x * b.y - b.x * a.y;
    }
    return std::abs(sum) * 0.5;
}

bool boundsFitInside(const AABB& inner, const AABB& outer, double eps) {
    if (!inner.isValid() || !outer.isValid()) {
        return false;
    }
    return inner.width() <= outer.width() + eps && inner.height() <= outer.height() + eps;
}

} // namespace

double cavityPlacementReward(const Document& document, const std::vector<Pose>& poses) {
    const size_t count = std::min(document.parts.size(), poses.size());
    if (count == 0 || !documentHasHoles(document)) {
        return 0.0;
    }

    std::vector<TransformedPart> transformed;
    transformed.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        transformed.push_back(transformPart(document.parts[i], poses[i], static_cast<int>(i)));
    }

    const double totalArea = std::max(1.0, document.totalPartArea());
    double reward = 0.0;
    for (size_t i = 0; i < count; ++i) {
        const AABB& partBounds = transformed[i].bounds;
        if (!partBounds.isValid()) {
            continue;
        }
        const Vec2 center = partBounds.center();
        const double partWeight = std::max(1.0, document.parts[i].area) / totalArea;
        for (size_t owner = 0; owner < count; ++owner) {
            if (owner == i) {
                continue;
            }
            for (const TransformedRing& ring : transformed[owner].rings) {
                if (!ring.isHole || !ring.bounds.overlaps(partBounds, 0.0) || !boundsFitInside(partBounds, ring.bounds, 1e-6)) {
                    continue;
                }
                const PointLocation location = pointInRing(ring.points, center, 1e-6);
                if (location == PointLocation::Outside) {
                    continue;
                }
                const double holeArea = std::max(1.0, ringArea(ring.points));
                reward += partWeight * std::min(1.0, document.parts[i].area / holeArea);
            }
        }
    }
    return reward;
}

} // namespace nest
