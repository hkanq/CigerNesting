#include "geometry/clearance.h"

#include "geometry/collision.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace nest {
namespace {

struct SegmentDistanceResult {
    double distance = std::numeric_limits<double>::max();
    Vec2 closestA;
    Vec2 closestB;
};

double clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
}

bool samePoint(Vec2 a, Vec2 b, double eps) {
    return distance(a, b) <= eps;
}

bool isClosedDuplicate(const std::vector<Vec2>& points, double eps) {
    return points.size() > 2 && samePoint(points.front(), points.back(), eps);
}

size_t effectivePointCount(const std::vector<Vec2>& points, double eps) {
    if (points.empty()) {
        return 0;
    }
    return isClosedDuplicate(points, eps) ? points.size() - 1 : points.size();
}

AABB boundsOfRing(const Ring& ring) {
    AABB box;
    for (const Vec2& point : ring.points) {
        box.include(point);
    }
    return box;
}

Ring rectangularOuter(const Sheet& sheet) {
    return sheet.makeRectangularOuterContour();
}

const Ring& customOuterOrFallbackStorage(const Sheet& sheet, Ring& fallback) {
    if (sheet.hasCustomProfile() && !sheet.profile().outerContour.points.empty()) {
        return sheet.profile().outerContour;
    }
    fallback = rectangularOuter(sheet);
    return fallback;
}

SegmentDistanceResult pointSegmentDistanceDetailed(Vec2 point, Vec2 a, Vec2 b) {
    const Vec2 ab = b - a;
    const double lengthSquared = ab.lengthSquared();
    if (lengthSquared <= std::numeric_limits<double>::epsilon()) {
        return {distance(point, a), point, a};
    }
    const double t = clamp01(dot(point - a, ab) / lengthSquared);
    const Vec2 closest = a + ab * t;
    return {distance(point, closest), point, closest};
}

SegmentDistanceResult segmentSegmentDistanceDetailed(Vec2 a, Vec2 b, Vec2 c, Vec2 d, double eps) {
    if (segmentsIntersect(a, b, c, d, eps)) {
        return {0.0, a, a};
    }

    SegmentDistanceResult best = pointSegmentDistanceDetailed(a, c, d);
    SegmentDistanceResult candidate = pointSegmentDistanceDetailed(b, c, d);
    if (candidate.distance < best.distance) {
        best = candidate;
    }

    candidate = pointSegmentDistanceDetailed(c, a, b);
    if (candidate.distance < best.distance) {
        best = {candidate.distance, candidate.closestB, candidate.closestA};
    }

    candidate = pointSegmentDistanceDetailed(d, a, b);
    if (candidate.distance < best.distance) {
        best = {candidate.distance, candidate.closestB, candidate.closestA};
    }
    return best;
}

template <typename Callback>
void forEachSegment(const std::vector<Vec2>& points, double eps, Callback callback) {
    const size_t count = effectivePointCount(points, eps);
    if (count < 2) {
        return;
    }
    for (size_t i = 0; i < count; ++i) {
        const Vec2 a = points[i];
        const Vec2 b = points[(i + 1) % count];
        if (!samePoint(a, b, eps)) {
            callback(a, b);
        }
    }
}

ClearanceResult initialResult(double requiredSpacing) {
    ClearanceResult result;
    result.valid = true;
    result.minDistance = std::numeric_limits<double>::max();
    result.ringA = -1;
    result.ringB = -1;
    (void)requiredSpacing;
    return result;
}

void updateResult(ClearanceResult& result, const SegmentDistanceResult& candidate, int ringA, int ringB, double requiredSpacing, double eps) {
    if (candidate.distance < result.minDistance) {
        result.minDistance = candidate.distance;
        result.closestA = candidate.closestA;
        result.closestB = candidate.closestB;
        result.ringA = ringA;
        result.ringB = ringB;
        result.valid = candidate.distance + eps >= requiredSpacing;
    }
}

ClearanceResult finalized(ClearanceResult result, double requiredSpacing, double eps) {
    if (result.minDistance == std::numeric_limits<double>::max()) {
        result.minDistance = std::numeric_limits<double>::infinity();
        result.valid = true;
        return result;
    }
    result.valid = result.minDistance + eps >= requiredSpacing;
    return result;
}

bool ringCanBeSkipped(const AABB& a, const AABB& b, double requiredSpacing, double eps) {
    if (requiredSpacing <= eps) {
        return false;
    }
    return !a.expanded(requiredSpacing + eps).overlaps(b.expanded(requiredSpacing + eps));
}

ClearanceResult minimumRingDistance(
    const std::vector<Vec2>& a,
    const AABB& boundsA,
    const std::vector<Vec2>& b,
    const AABB& boundsB,
    int ringA,
    int ringB,
    double requiredSpacing,
    double eps) {
    ClearanceResult result = initialResult(requiredSpacing);
    if (a.size() < 2 || b.size() < 2 || ringCanBeSkipped(boundsA, boundsB, requiredSpacing, eps)) {
        return finalized(result, requiredSpacing, eps);
    }

    bool earlyInvalid = false;
    forEachSegment(a, eps, [&](Vec2 a0, Vec2 a1) {
        if (earlyInvalid) {
            return;
        }
        const AABB segA = AABB::fromMinMax(
            {std::min(a0.x, a1.x), std::min(a0.y, a1.y)},
            {std::max(a0.x, a1.x), std::max(a0.y, a1.y)});
        forEachSegment(b, eps, [&](Vec2 b0, Vec2 b1) {
            if (earlyInvalid) {
                return;
            }
            const AABB segB = AABB::fromMinMax(
                {std::min(b0.x, b1.x), std::min(b0.y, b1.y)},
                {std::max(b0.x, b1.x), std::max(b0.y, b1.y)});
            if (ringCanBeSkipped(segA, segB, std::min(requiredSpacing, result.minDistance), eps)) {
                return;
            }
            const SegmentDistanceResult distanceResult = segmentSegmentDistanceDetailed(a0, a1, b0, b1, eps);
            updateResult(result, distanceResult, ringA, ringB, requiredSpacing, eps);
            if (result.minDistance + eps < requiredSpacing) {
                earlyInvalid = true;
            }
        });
    });
    return finalized(result, requiredSpacing, eps);
}

bool anyPartBoundaryCloseToRings(const TransformedPart& transformed, const std::vector<Ring>& rings, double margin, double eps) {
    for (const Ring& ring : rings) {
        const ClearanceResult result = minimumPartToRingDistance(transformed, ring, margin, eps);
        if (!result.valid) {
            return true;
        }
    }
    return false;
}

} // namespace

ClearanceResult minimumBoundaryDistance(
    const TransformedPart& a,
    const TransformedPart& b,
    double requiredSpacing,
    double eps) {
    ClearanceResult best = initialResult(requiredSpacing);
    if (!a.bounds.expanded(requiredSpacing + eps).overlaps(b.bounds.expanded(requiredSpacing + eps))) {
        return finalized(best, requiredSpacing, eps);
    }

    for (size_t ringA = 0; ringA < a.rings.size(); ++ringA) {
        const TransformedRing& aRing = a.rings[ringA];
        for (size_t ringB = 0; ringB < b.rings.size(); ++ringB) {
            const TransformedRing& bRing = b.rings[ringB];
            ClearanceResult local = minimumRingDistance(
                aRing.points,
                aRing.bounds,
                bRing.points,
                bRing.bounds,
                static_cast<int>(ringA),
                static_cast<int>(ringB),
                requiredSpacing,
                eps);
            if (local.minDistance < best.minDistance) {
                best = local;
            }
            if (!best.valid) {
                return best;
            }
        }
    }
    return finalized(best, requiredSpacing, eps);
}

ClearanceResult minimumPartToRingDistance(
    const TransformedPart& part,
    const Ring& ring,
    double requiredSpacing,
    double eps) {
    ClearanceResult best = initialResult(requiredSpacing);
    const AABB ringBounds = boundsOfRing(ring);
    if (!part.bounds.expanded(requiredSpacing + eps).overlaps(ringBounds.expanded(requiredSpacing + eps))) {
        return finalized(best, requiredSpacing, eps);
    }

    for (size_t ringIndex = 0; ringIndex < part.rings.size(); ++ringIndex) {
        const TransformedRing& partRing = part.rings[ringIndex];
        ClearanceResult local = minimumRingDistance(
            partRing.points,
            partRing.bounds,
            ring.points,
            ringBounds,
            static_cast<int>(ringIndex),
            0,
            requiredSpacing,
            eps);
        if (local.minDistance < best.minDistance) {
            best = local;
        }
        if (!best.valid) {
            return best;
        }
    }
    return finalized(best, requiredSpacing, eps);
}

bool partsRespectClearance(
    const Part& a,
    const Pose& poseA,
    const Part& b,
    const Pose& poseB,
    double requiredSpacing,
    double eps) {
    if (requiredSpacing <= eps) {
        return true;
    }
    const TransformedPart partA = transformPart(a, poseA, 0);
    const TransformedPart partB = transformPart(b, poseB, 1);
    return minimumBoundaryDistance(partA, partB, requiredSpacing, eps).valid;
}

bool partRespectsSheetClearance(
    const Part& part,
    const Pose& pose,
    const Sheet& sheet,
    double margin,
    double eps) {
    if (!isPartInsideSheet(part, pose, sheet, eps) ||
        overlapsSheetHolesOrForbiddenZones(part, pose, sheet, eps)) {
        return false;
    }
    if (margin <= eps) {
        return true;
    }

    const TransformedPart transformed = transformPart(part, pose);
    Ring fallbackOuter;
    const Ring& outer = customOuterOrFallbackStorage(sheet, fallbackOuter);
    if (!minimumPartToRingDistance(transformed, outer, margin, eps).valid) {
        return false;
    }
    if (anyPartBoundaryCloseToRings(transformed, sheet.profile().holes, margin, eps) ||
        anyPartBoundaryCloseToRings(transformed, sheet.profile().forbiddenZones, margin, eps)) {
        return false;
    }
    return true;
}

} // namespace nest
