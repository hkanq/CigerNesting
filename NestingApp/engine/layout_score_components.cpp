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

LayoutShapeMetrics computeLayoutShapeMetrics(const Document& document, const EngineSettings& settings, const AABB& usedBounds) {
    LayoutShapeMetrics metrics;
    if (!usedBounds.isValid() || usedBounds.width() <= 1e-9 || usedBounds.height() <= 1e-9) {
        return metrics;
    }

    const double sheetWidth = std::max(1.0, (document.sheet.width > 0.0 ? document.sheet.width : settings.sheetWidth) - settings.margin * 2.0);
    const double sheetHeight = std::max(1.0, (document.sheet.height > 0.0 ? document.sheet.height : settings.sheetHeight) - settings.margin * 2.0);
    const double usedWidth = std::max(1.0, usedBounds.width());
    const double usedHeight = std::max(1.0, usedBounds.height());
    const double usedAspect = std::max(usedWidth / usedHeight, usedHeight / usedWidth);
    const double sheetAspect = std::max(sheetWidth / sheetHeight, sheetHeight / sheetWidth);
    const double aspectAllowance = std::max(2.15, sheetAspect * 1.65);

    metrics.towerScore = std::max(0.0, usedAspect / aspectAllowance - 1.0);

    const double widthCoverage = std::min(1.0, usedWidth / sheetWidth);
    const double heightCoverage = std::min(1.0, usedHeight / sheetHeight);
    const double narrowCoverage = std::min(widthCoverage, heightCoverage);
    const double wideCoverage = std::max(widthCoverage, heightCoverage);
    if (wideCoverage > 0.58 && narrowCoverage < 0.24) {
        metrics.towerScore += (0.24 - narrowCoverage) * 2.0 * wideCoverage;
    }

    // Good spread here means "not a needle". It is intentionally mild so a compact corner cluster is still allowed.
    metrics.layoutSpreadScore = std::sqrt(std::max(0.0, widthCoverage * heightCoverage));
    const double sheetArea = std::max(1.0, sheetWidth * sheetHeight);
    metrics.unusedSheetRegionScore = std::max(0.0, 1.0 - (usedWidth * usedHeight) / sheetArea);
    return metrics;
}

} // namespace nest
