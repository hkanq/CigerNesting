#include "geometry/polygon_offset.h"

#include "geometry/minkowski_sum.h"
#include <algorithm>
#include <cmath>

namespace nest {
namespace {

Vec2 normalized(Vec2 v) {
    const double len = v.length();
    return len > 1e-12 ? v / len : Vec2{};
}

Vec2 outwardNormal(Vec2 edge, double orientation) {
    edge = normalized(edge);
    return orientation >= 0.0 ? Vec2{edge.y, -edge.x} : Vec2{-edge.y, edge.x};
}

} // namespace

Ring offsetRingConservative(const Ring& ring, const PolygonOffsetOptions& options) {
    Ring out;
    out.isHole = ring.isHole;
    std::vector<Vec2> points = cleanPolygonPoints(ring.points, options.tolerance);
    if (points.size() < 3 || std::abs(options.distance) <= options.tolerance) {
        out.points = std::move(points);
        out.winding = polygonSignedArea(out.points) >= 0.0 ? 1 : -1;
        return out;
    }
    const double orientation = polygonSignedArea(points) >= 0.0 ? 1.0 : -1.0;
    out.points.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        const Vec2 prev = points[(i + points.size() - 1) % points.size()];
        const Vec2 cur = points[i];
        const Vec2 next = points[(i + 1) % points.size()];
        const Vec2 n1 = outwardNormal(cur - prev, orientation);
        const Vec2 n2 = outwardNormal(next - cur, orientation);
        Vec2 bisector = n1 + n2;
        if (bisector.length() <= 1e-12) {
            bisector = n2;
        }
        bisector = normalized(bisector);
        double scale = options.distance;
        const double denom = dot(bisector, n2);
        if (std::abs(denom) > 1e-6) {
            scale = options.distance / denom;
            const double maxScale = std::abs(options.distance) * std::max(1.0, options.miterLimit);
            scale = std::clamp(scale, -maxScale, maxScale);
        }
        out.points.push_back(cur + bisector * scale);
    }
    out.winding = polygonSignedArea(out.points) >= 0.0 ? 1 : -1;
    return out;
}

std::vector<Ring> offsetRingsConservative(const std::vector<Ring>& rings, const PolygonOffsetOptions& options) {
    std::vector<Ring> out;
    out.reserve(rings.size());
    for (const Ring& ring : rings) {
        PolygonOffsetOptions local = options;
        if (ring.isHole) {
            local.distance = -local.distance;
        }
        out.push_back(offsetRingConservative(ring, local));
    }
    return out;
}

} // namespace nest
