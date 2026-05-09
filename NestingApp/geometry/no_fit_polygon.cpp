#include "geometry/no_fit_polygon.h"

#include "geometry/polygon_offset.h"
#include "geometry/transformed_shape.h"
#include <algorithm>
#include <cmath>

namespace nest {
namespace {

AABB boundsOfPoints(const std::vector<Vec2>& points) {
    AABB box;
    for (Vec2 p : points) {
        box.include(p);
    }
    return box;
}

Ring transformedLocalRing(const Ring& ring, const Pose& orientation) {
    Ring out = ring;
    Pose local = orientation;
    local.x = 0.0;
    local.y = 0.0;
    const Transform transform = local.toTransform();
    for (Vec2& p : out.points) {
        p = transform.apply(p);
    }
    out.winding = polygonSignedArea(out.points) >= 0.0 ? 1 : -1;
    return out;
}

void appendLoop(NoFitPolygonResult& result, Ring ring, bool fromHole, bool exactConvex) {
    ring.points = cleanPolygonPoints(ring.points);
    if (ring.points.size() < 3) {
        return;
    }
    NoFitPolygonLoop loop;
    loop.bounds = boundsOfPoints(ring.points);
    loop.fromHole = fromHole;
    loop.exactConvex = exactConvex;
    loop.ring = std::move(ring);
    result.loops.push_back(std::move(loop));
}

} // namespace

NoFitPolygonResult buildNoFitPolygon(
    const Part& moving,
    const Pose& movingOrientation,
    const Part& fixed,
    const Pose& fixedOrientation,
    const NoFitPolygonOptions& options) {
    NoFitPolygonResult result;
    for (const Ring& fixedRingOriginal : fixed.rings) {
        if (fixedRingOriginal.isHole && !options.includeHoles) {
            continue;
        }
        Ring fixedRing = transformedLocalRing(fixedRingOriginal, fixedOrientation);
        if (std::abs(options.spacing) > options.tolerance) {
            fixedRing = offsetRingConservative(fixedRing, {options.spacing, options.tolerance, OffsetJoinStyle::Miter, 4.0});
        }
        for (const Ring& movingRingOriginal : moving.rings) {
            if (movingRingOriginal.isHole) {
                continue;
            }
            Ring movingRing = transformedLocalRing(movingRingOriginal, movingOrientation);
            Ring reflectedMoving = reflectedRing(movingRing);
            MinkowskiResult sum = minkowskiSumDecomposed(fixedRing, reflectedMoving, options.tolerance);
            result.usedDecomposition = result.usedDecomposition || sum.usedDecomposition;
            for (MinkowskiComponent& component : sum.components) {
                appendLoop(result, std::move(component.ring), fixedRingOriginal.isHole, component.exactConvex);
            }
        }
    }
    result.componentCount = result.loops.size();
    return result;
}

std::vector<Pose> sampleNoFitPolygonCandidates(
    const NoFitPolygonResult& nfp,
    const Pose& orientation,
    size_t limit,
    Vec2 target) {
    std::vector<Pose> candidates;
    auto append = [&](Vec2 p) {
        Pose pose = orientation;
        pose.x = p.x;
        pose.y = p.y;
        const bool exists = std::any_of(candidates.begin(), candidates.end(), [&](const Pose& existing) {
            return std::abs(existing.x - pose.x) <= 1e-5 && std::abs(existing.y - pose.y) <= 1e-5 &&
                std::abs(existing.angleRadians - pose.angleRadians) <= 1e-8 && existing.mirrored == pose.mirrored;
        });
        if (!exists) {
            candidates.push_back(pose);
        }
    };
    for (const NoFitPolygonLoop& loop : nfp.loops) {
        const std::vector<Vec2> points = cleanPolygonPoints(loop.ring.points);
        if (points.empty()) {
            continue;
        }
        size_t minX = 0, maxX = 0, minY = 0, maxY = 0, closest = 0;
        for (size_t i = 1; i < points.size(); ++i) {
            if (points[i].x < points[minX].x) minX = i;
            if (points[i].x > points[maxX].x) maxX = i;
            if (points[i].y < points[minY].y) minY = i;
            if (points[i].y > points[maxY].y) maxY = i;
            if (distance(points[i], target) < distance(points[closest], target)) closest = i;
        }
        append(points[minX]);
        append(points[maxX]);
        append(points[minY]);
        append(points[maxY]);
        append(points[closest]);
        const size_t stride = std::max<size_t>(1, points.size() / 8u);
        for (size_t i = 0; i < points.size(); i += stride) {
            append(points[i]);
            const Vec2 mid = (points[i] + points[(i + 1) % points.size()]) * 0.5;
            append(mid);
            if (candidates.size() >= limit) {
                return candidates;
            }
        }
    }
    if (candidates.size() > limit) {
        candidates.resize(limit);
    }
    return candidates;
}

} // namespace nest
