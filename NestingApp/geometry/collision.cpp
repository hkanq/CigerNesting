#include "geometry/collision.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nest {
namespace {

double sqr(double value) {
    return value * value;
}

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

AABB boundsOfPoints(const std::vector<Vec2>& points) {
    AABB box;
    for (const Vec2& point : points) {
        box.include(point);
    }
    return box;
}

double orientation(Vec2 a, Vec2 b, Vec2 c) {
    return cross(b - a, c - a);
}

double pointSegmentDistance(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 ab = b - a;
    const double lengthSquared = ab.lengthSquared();
    if (lengthSquared <= std::numeric_limits<double>::epsilon()) {
        return distance(p, a);
    }
    const double t = clamp01(dot(p - a, ab) / lengthSquared);
    return distance(p, a + ab * t);
}

bool pointOnSegment(Vec2 p, Vec2 a, Vec2 b, double eps) {
    return pointSegmentDistance(p, a, b) <= eps &&
        p.x >= std::min(a.x, b.x) - eps && p.x <= std::max(a.x, b.x) + eps &&
        p.y >= std::min(a.y, b.y) - eps && p.y <= std::max(a.y, b.y) + eps;
}

int orientationSign(Vec2 a, Vec2 b, Vec2 c, double eps) {
    const double value = orientation(a, b, c);
    const double scale = std::max(1.0, (b - a).length() * (c - a).length());
    const double threshold = eps * scale;
    if (value > threshold) {
        return 1;
    }
    if (value < -threshold) {
        return -1;
    }
    return 0;
}

bool segmentsProperlyIntersect(Vec2 a, Vec2 b, Vec2 c, Vec2 d, double eps) {
    const int o1 = orientationSign(a, b, c, eps);
    const int o2 = orientationSign(a, b, d, eps);
    const int o3 = orientationSign(c, d, a, eps);
    const int o4 = orientationSign(c, d, b, eps);
    return o1 != 0 && o2 != 0 && o3 != 0 && o4 != 0 && o1 != o2 && o3 != o4;
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

double segmentDistance(Vec2 a, Vec2 b, Vec2 c, Vec2 d, double eps) {
    if (segmentsIntersect(a, b, c, d, eps)) {
        return 0.0;
    }
    return std::min({
        pointSegmentDistance(a, c, d),
        pointSegmentDistance(b, c, d),
        pointSegmentDistance(c, a, b),
        pointSegmentDistance(d, a, b)
    });
}

bool anyRingSegmentsIntersect(const std::vector<Vec2>& a, const std::vector<Vec2>& b, double eps) {
    bool intersects = false;
    forEachSegment(a, eps, [&](Vec2 a0, Vec2 a1) {
        if (intersects) {
            return;
        }
        forEachSegment(b, eps, [&](Vec2 b0, Vec2 b1) {
            if (!intersects && segmentsIntersect(a0, a1, b0, b1, eps)) {
                intersects = true;
            }
        });
    });
    return intersects;
}

bool ringContainsOrTouchesPoint(const std::vector<Vec2>& ring, Vec2 point, double eps) {
    const PointLocation location = pointInRing(ring, point, eps);
    return location == PointLocation::Inside || location == PointLocation::OnBoundary;
}

bool isPointInAnyZone(const std::vector<Ring>& zones, Vec2 point, double eps) {
    for (const Ring& zone : zones) {
        if (ringContainsOrTouchesPoint(zone.points, point, eps)) {
            return true;
        }
    }
    return false;
}

const std::vector<Ring>& sheetHoles(const Sheet& sheet);
const std::vector<Ring>& sheetForbiddenZones(const Sheet& sheet);

bool ringSegmentProperlyCrossesBoundary(Vec2 a, Vec2 b, const std::vector<Vec2>& boundary, double eps) {
    bool crosses = false;
    forEachSegment(boundary, eps, [&](Vec2 c, Vec2 d) {
        if (!crosses && segmentsProperlyIntersect(a, b, c, d, eps)) {
            crosses = true;
        }
    });
    return crosses;
}

bool segmentSamplesInsideUsableSheet(Vec2 a, Vec2 b, const Ring& outer, const Sheet& sheet, double eps) {
    constexpr double fractions[] = {0.0, 0.0625, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 0.9375, 1.0};
    for (const double t : fractions) {
        const Vec2 point = a + (b - a) * t;
        if (pointInRing(outer.points, point, eps) == PointLocation::Outside) {
            return false;
        }
        if (isPointInAnyZone(sheetHoles(sheet), point, eps) ||
            isPointInAnyZone(sheetForbiddenZones(sheet), point, eps)) {
            return false;
        }
    }
    return true;
}

Ring rectangularOuterWithMargin(const Sheet& sheet, double margin) {
    const double left = sheet.origin.x + margin;
    const double top = sheet.origin.y + margin;
    const double right = sheet.origin.x + sheet.width - margin;
    const double bottom = sheet.origin.y + sheet.height - margin;
    Ring ring;
    ring.points = {
        {left, top},
        {right, top},
        {right, bottom},
        {left, bottom},
        {left, top}
    };
    ring.isHole = false;
    ring.winding = 1;
    return ring;
}

Ring usableSheetOuter(const Sheet& sheet) {
    if (sheet.hasCustomProfile() && !sheet.profile().outerContour.points.empty()) {
        return sheet.profile().outerContour;
    }
    return rectangularOuterWithMargin(sheet, sheet.margin);
}

Ring physicalSheetOuter(const Sheet& sheet) {
    if (sheet.hasCustomProfile() && !sheet.profile().outerContour.points.empty()) {
        return sheet.profile().outerContour;
    }
    return sheet.makeRectangularOuterContour();
}

const std::vector<Ring>& sheetHoles(const Sheet& sheet) {
    return sheet.profile().holes;
}

const std::vector<Ring>& sheetForbiddenZones(const Sheet& sheet) {
    return sheet.profile().forbiddenZones;
}

bool zoneOverlapsSolidPart(const TransformedPart& part, const Ring& zone, double eps) {
    if (zone.points.size() < 3) {
        return false;
    }

    const AABB zoneBounds = boundsOfPoints(zone.points);
    if (!part.bounds.expanded(eps).overlaps(zoneBounds.expanded(eps))) {
        return false;
    }

    for (const TransformedRing& ring : part.rings) {
        if (ring.isHole || ring.points.size() < 3) {
            continue;
        }
        if (anyRingSegmentsIntersect(ring.points, zone.points, eps)) {
            return true;
        }
        for (const Vec2& point : ring.points) {
            if (ringContainsOrTouchesPoint(zone.points, point, eps)) {
                return true;
            }
        }
    }

    for (const Vec2& point : zone.points) {
        if (pointInSolidArea(part, point, eps)) {
            return true;
        }
    }
    return false;
}

bool zonesOverlapSolidPart(const TransformedPart& part, const std::vector<Ring>& zones, double eps) {
    for (const Ring& zone : zones) {
        if (zoneOverlapsSolidPart(part, zone, eps)) {
            return true;
        }
    }
    return false;
}

bool anySolidSampleInsideOther(const TransformedPart& source, const TransformedPart& other, double eps) {
    for (const TransformedRing& ring : source.rings) {
        if (ring.isHole || ring.points.empty()) {
            continue;
        }
        const size_t count = effectivePointCount(ring.points, eps);
        for (size_t i = 0; i < count; ++i) {
            const Vec2 a = ring.points[i];
            const Vec2 b = ring.points[(i + 1) % count];
            if (pointInSolidArea(other, a, eps)) {
                return true;
            }
            const Vec2 mid = (a + b) * 0.5;
            if (pointInSolidArea(other, mid, eps)) {
                return true;
            }
        }
    }
    return false;
}

double minRingDistance(const std::vector<Vec2>& a, const std::vector<Vec2>& b, double eps) {
    double best = std::numeric_limits<double>::max();
    forEachSegment(a, eps, [&](Vec2 a0, Vec2 a1) {
        forEachSegment(b, eps, [&](Vec2 b0, Vec2 b1) {
            best = std::min(best, segmentDistance(a0, a1, b0, b1, eps));
        });
    });
    return best;
}

double minBoundaryDistance(const TransformedPart& a, const TransformedPart& b, double eps) {
    double best = std::numeric_limits<double>::max();
    for (const TransformedRing& ringA : a.rings) {
        if (ringA.points.size() < 2) {
            continue;
        }
        for (const TransformedRing& ringB : b.rings) {
            if (ringB.points.size() < 2) {
                continue;
            }
            best = std::min(best, minRingDistance(ringA.points, ringB.points, eps));
        }
    }
    return best;
}

double minBoundaryDistanceToRing(const TransformedPart& part, const Ring& ring, double eps) {
    if (ring.points.size() < 2) {
        return std::numeric_limits<double>::max();
    }
    double best = std::numeric_limits<double>::max();
    for (const TransformedRing& partRing : part.rings) {
        if (partRing.points.size() < 2) {
            continue;
        }
        best = std::min(best, minRingDistance(partRing.points, ring.points, eps));
    }
    return best;
}

bool transformedPartInsideSheet(const TransformedPart& part, const Sheet& sheet, double eps) {
    const Ring outer = usableSheetOuter(sheet);
    if (outer.points.size() < 3) {
        return false;
    }

    for (const TransformedRing& ring : part.rings) {
        if (ring.isHole || ring.points.size() < 3) {
            continue;
        }

        const size_t count = effectivePointCount(ring.points, eps);
        for (size_t i = 0; i < count; ++i) {
            const Vec2 point = ring.points[i];
            if (pointInRing(outer.points, point, eps) == PointLocation::Outside) {
                return false;
            }
            if (isPointInAnyZone(sheetHoles(sheet), point, eps) ||
                isPointInAnyZone(sheetForbiddenZones(sheet), point, eps)) {
                return false;
            }

            const Vec2 next = ring.points[(i + 1) % count];
            if (!segmentSamplesInsideUsableSheet(point, next, outer, sheet, eps)) {
                return false;
            }
            if (ringSegmentProperlyCrossesBoundary(point, next, outer.points, eps)) {
                return false;
            }
        }
    }

    if (zonesOverlapSolidPart(part, sheetHoles(sheet), eps) ||
        zonesOverlapSolidPart(part, sheetForbiddenZones(sheet), eps)) {
        return false;
    }

    return true;
}

} // namespace

bool segmentsIntersect(Vec2 a, Vec2 b, Vec2 c, Vec2 d, double eps) {
    if (!AABB::fromMinMax({std::min(a.x, b.x), std::min(a.y, b.y)}, {std::max(a.x, b.x), std::max(a.y, b.y)})
             .expanded(eps)
             .overlaps(AABB::fromMinMax({std::min(c.x, d.x), std::min(c.y, d.y)}, {std::max(c.x, d.x), std::max(c.y, d.y)}).expanded(eps))) {
        return false;
    }

    if (pointOnSegment(a, c, d, eps) || pointOnSegment(b, c, d, eps) ||
        pointOnSegment(c, a, b, eps) || pointOnSegment(d, a, b, eps)) {
        return true;
    }

    const int o1 = orientationSign(a, b, c, eps);
    const int o2 = orientationSign(a, b, d, eps);
    const int o3 = orientationSign(c, d, a, eps);
    const int o4 = orientationSign(c, d, b, eps);
    return o1 != 0 && o2 != 0 && o3 != 0 && o4 != 0 && o1 != o2 && o3 != o4;
}

PointLocation pointInRing(const std::vector<Vec2>& ring, Vec2 p, double eps) {
    const size_t count = effectivePointCount(ring, eps);
    if (count < 3) {
        return PointLocation::Outside;
    }

    for (size_t i = 0; i < count; ++i) {
        if (pointOnSegment(p, ring[i], ring[(i + 1) % count], eps)) {
            return PointLocation::OnBoundary;
        }
    }

    bool inside = false;
    for (size_t i = 0, j = count - 1; i < count; j = i++) {
        const Vec2 pi = ring[i];
        const Vec2 pj = ring[j];
        const bool crosses = ((pi.y > p.y) != (pj.y > p.y)) &&
            (p.x < (pj.x - pi.x) * (p.y - pi.y) / ((pj.y - pi.y) + 1e-30) + pi.x);
        if (crosses) {
            inside = !inside;
        }
    }

    return inside ? PointLocation::Inside : PointLocation::Outside;
}

PointLocation pointInRing(const Ring& ring, Vec2 p, double eps) {
    return pointInRing(ring.points, p, eps);
}

bool ringsOverlapOrTouch(const std::vector<Vec2>& a, const std::vector<Vec2>& b, double eps) {
    if (effectivePointCount(a, eps) < 3 || effectivePointCount(b, eps) < 3) {
        return false;
    }
    if (!boundsOfPoints(a).expanded(eps).overlaps(boundsOfPoints(b).expanded(eps))) {
        return false;
    }
    if (anyRingSegmentsIntersect(a, b, eps)) {
        return true;
    }
    return pointInRing(b, a.front(), eps) != PointLocation::Outside ||
        pointInRing(a, b.front(), eps) != PointLocation::Outside;
}

bool ringsOverlapOrTouch(const TransformedRing& a, const TransformedRing& b, double eps) {
    if (!a.bounds.expanded(eps).overlaps(b.bounds.expanded(eps))) {
        return false;
    }
    return ringsOverlapOrTouch(a.points, b.points, eps);
}

bool pointInSolidArea(const TransformedPart& part, Vec2 p, double eps) {
    bool inOuter = false;
    for (const TransformedRing& ring : part.rings) {
        if (ring.isHole) {
            continue;
        }
        const PointLocation location = pointInRing(ring.points, p, eps);
        if (location == PointLocation::Inside || location == PointLocation::OnBoundary) {
            inOuter = true;
            break;
        }
    }
    if (!inOuter) {
        return false;
    }

    for (const TransformedRing& ring : part.rings) {
        if (!ring.isHole) {
            continue;
        }
        const PointLocation location = pointInRing(ring.points, p, eps);
        if (location == PointLocation::Inside) {
            return false;
        }
        if (location == PointLocation::OnBoundary) {
            return true;
        }
    }
    return true;
}

bool partsCollide(const Part& a, const Pose& poseA, const Part& b, const Pose& poseB, double eps) {
    const TransformedPart partA = transformPart(a, poseA, 0);
    const TransformedPart partB = transformPart(b, poseB, 1);
    if (!partA.bounds.expanded(eps).overlaps(partB.bounds.expanded(eps))) {
        return false;
    }

    for (const TransformedRing& ringA : partA.rings) {
        if (ringA.points.size() < 2) {
            continue;
        }
        for (const TransformedRing& ringB : partB.rings) {
            if (ringB.points.size() < 2) {
                continue;
            }
            if (ringA.bounds.expanded(eps).overlaps(ringB.bounds.expanded(eps)) &&
                anyRingSegmentsIntersect(ringA.points, ringB.points, eps)) {
                return true;
            }
        }
    }

    return anySolidSampleInsideOther(partA, partB, eps) || anySolidSampleInsideOther(partB, partA, eps);
}

bool partsRespectSpacing(const Part& a, const Pose& poseA, const Part& b, const Pose& poseB, const ClearanceSettings& clearance) {
    const double eps = clearance.tolerance;
    if (partsCollide(a, poseA, b, poseB, eps)) {
        return false;
    }
    if (clearance.partSpacing <= eps) {
        return true;
    }

    const TransformedPart partA = transformPart(a, poseA, 0);
    const TransformedPart partB = transformPart(b, poseB, 1);
    if (!partA.bounds.expanded(clearance.partSpacing + eps).overlaps(partB.bounds.expanded(clearance.partSpacing + eps))) {
        return true;
    }
    return minBoundaryDistance(partA, partB, eps) + eps >= clearance.partSpacing;
}

bool isPartInsideSheet(const Part& part, const Pose& pose, const Sheet& sheet, double eps) {
    return transformedPartInsideSheet(transformPart(part, pose), sheet, eps);
}

bool overlapsSheetHolesOrForbiddenZones(const Part& part, const Pose& pose, const Sheet& sheet, double eps) {
    const TransformedPart transformed = transformPart(part, pose);
    return zonesOverlapSolidPart(transformed, sheetHoles(sheet), eps) ||
        zonesOverlapSolidPart(transformed, sheetForbiddenZones(sheet), eps);
}

double minimumDistanceToSheetFeatures(const Part& part, const Pose& pose, const Sheet& sheet, double eps) {
    const TransformedPart transformed = transformPart(part, pose);
    double best = minBoundaryDistanceToRing(transformed, physicalSheetOuter(sheet), eps);
    for (const Ring& hole : sheetHoles(sheet)) {
        best = std::min(best, minBoundaryDistanceToRing(transformed, hole, eps));
    }
    for (const Ring& zone : sheetForbiddenZones(sheet)) {
        best = std::min(best, minBoundaryDistanceToRing(transformed, zone, eps));
    }
    return best;
}

bool partRespectsSheetMargin(const Part& part, const Pose& pose, const Sheet& sheet, const ClearanceSettings& clearance) {
    const double eps = clearance.tolerance;
    if (!isPartInsideSheet(part, pose, sheet, eps)) {
        return false;
    }
    if (clearance.sheetMargin <= eps) {
        return true;
    }

    const TransformedPart transformed = transformPart(part, pose);
    const Ring outer = physicalSheetOuter(sheet);
    if (minBoundaryDistanceToRing(transformed, outer, eps) + eps < clearance.sheetMargin) {
        return false;
    }
    for (const Ring& hole : sheetHoles(sheet)) {
        if (minBoundaryDistanceToRing(transformed, hole, eps) + eps < clearance.sheetMargin) {
            return false;
        }
    }
    for (const Ring& zone : sheetForbiddenZones(sheet)) {
        if (minBoundaryDistanceToRing(transformed, zone, eps) + eps < clearance.sheetMargin) {
            return false;
        }
    }
    return true;
}

bool ringsOverlap(const Ring& a, const Ring& b, double tolerance) {
    return ringsOverlapOrTouch(a.points, b.points, tolerance);
}

bool partsOverlap(const Part& a, const Pose& poseA, const Part& b, const Pose& poseB, double tolerance) {
    return partsCollide(a, poseA, b, poseB, tolerance);
}

bool isPartInsideSheet(const Part& part, const Pose& pose, const Sheet& sheet) {
    return isPartInsideSheet(part, pose, sheet, 1e-6);
}

bool overlapsSheetHolesOrForbiddenZones(const Part& part, const Pose& pose, const Sheet& sheet) {
    return overlapsSheetHolesOrForbiddenZones(part, pose, sheet, 1e-6);
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
