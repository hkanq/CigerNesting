#include "geometry/inner_fit_polygon.h"

#include "geometry/minkowski_sum.h"
#include "geometry/polygon_utils.h"
#include <algorithm>

namespace nest {
namespace {

AABB boundsOfRingPoints(const Ring& ring) {
    AABB box;
    for (Vec2 p : ring.points) {
        box.include(p);
    }
    return box;
}

Ring makeRingFromBox(double left, double bottom, double right, double top) {
    Ring ring;
    ring.points = {{left, bottom}, {right, bottom}, {right, top}, {left, top}};
    ring.winding = 1;
    return ring;
}

} // namespace

InnerFitPolygonResult buildInnerFitPolygon(
    const Part& moving,
    const Pose& movingOrientation,
    const Sheet& sheet,
    const InnerFitPolygonOptions& options) {
    InnerFitPolygonResult result;
    Pose orientation = movingOrientation;
    orientation.x = 0.0;
    orientation.y = 0.0;
    const AABB movingBounds = transformedBounds(moving, orientation);
    if (!movingBounds.isValid()) {
        return result;
    }

    if (!sheet.hasCustomProfile()) {
        const double left = sheet.origin.x + options.margin - movingBounds.min.x;
        const double bottom = sheet.origin.y + options.margin - movingBounds.min.y;
        const double right = sheet.origin.x + sheet.width - options.margin - movingBounds.max.x;
        const double top = sheet.origin.y + sheet.height - options.margin - movingBounds.max.y;
        if (left <= right + options.tolerance && bottom <= top + options.tolerance) {
            InnerFitPolygonLoop loop;
            loop.ring = makeRingFromBox(left, bottom, right, top);
            loop.bounds = boundsOfRingPoints(loop.ring);
            loop.exactRectangular = true;
            result.loops.push_back(loop);
            result.exactRectangular = true;
        }
        return result;
    }

    Ring outer = sheet.profile().outerContour;
    if (!outer.points.empty()) {
        InnerFitPolygonLoop loop;
        loop.ring = outer;
        loop.bounds = boundsOfRingPoints(loop.ring);
        loop.exactRectangular = false;
        result.loops.push_back(loop);
    }
    return result;
}

std::vector<Pose> sampleInnerFitPolygonCandidates(
    const InnerFitPolygonResult& ifp,
    const Pose& orientation,
    size_t limit,
    Vec2 target) {
    std::vector<Pose> candidates;
    auto append = [&](Vec2 p) {
        Pose pose = orientation;
        pose.x = p.x;
        pose.y = p.y;
        const bool exists = std::any_of(candidates.begin(), candidates.end(), [&](const Pose& existing) {
            return std::abs(existing.x - pose.x) <= 1e-5 && std::abs(existing.y - pose.y) <= 1e-5 && existing.mirrored == pose.mirrored;
        });
        if (!exists) {
            candidates.push_back(pose);
        }
    };
    for (const InnerFitPolygonLoop& loop : ifp.loops) {
        const std::vector<Vec2> points = cleanPolygonPoints(loop.ring.points);
        if (points.empty()) {
            continue;
        }
        append(loop.bounds.min);
        append({loop.bounds.max.x, loop.bounds.min.y});
        append(loop.bounds.max);
        append({loop.bounds.min.x, loop.bounds.max.y});
        append(loop.bounds.center());
        size_t closest = 0;
        for (size_t i = 1; i < points.size(); ++i) {
            if (distance(points[i], target) < distance(points[closest], target)) {
                closest = i;
            }
        }
        append(points[closest]);
        for (size_t i = 0; i < points.size(); ++i) {
            append((points[i] + points[(i + 1) % points.size()]) * 0.5);
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
