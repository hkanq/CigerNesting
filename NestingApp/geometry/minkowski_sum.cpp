#include "geometry/minkowski_sum.h"

#include <algorithm>
#include <cmath>

namespace nest {
namespace {

bool pointInTriangle(Vec2 p, Vec2 a, Vec2 b, Vec2 c, double eps) {
    const double c1 = cross(b - a, p - a);
    const double c2 = cross(c - b, p - b);
    const double c3 = cross(a - c, p - c);
    const bool hasNeg = c1 < -eps || c2 < -eps || c3 < -eps;
    const bool hasPos = c1 > eps || c2 > eps || c3 > eps;
    return !(hasNeg && hasPos);
}

std::vector<Vec2> convexHull(std::vector<Vec2> points, double eps) {
    points = cleanPolygonPoints(points, eps);
    std::sort(points.begin(), points.end(), [](Vec2 a, Vec2 b) {
        if (std::abs(a.x - b.x) > 1e-9) {
            return a.x < b.x;
        }
        return a.y < b.y;
    });
    std::vector<Vec2> unique;
    for (Vec2 p : points) {
        if (unique.empty() || distance(unique.back(), p) > eps) {
            unique.push_back(p);
        }
    }
    if (unique.size() <= 2) {
        return unique;
    }
    std::vector<Vec2> lower;
    for (Vec2 p : unique) {
        while (lower.size() >= 2 && cross(lower.back() - lower[lower.size() - 2], p - lower.back()) <= eps) {
            lower.pop_back();
        }
        lower.push_back(p);
    }
    std::vector<Vec2> upper;
    for (auto it = unique.rbegin(); it != unique.rend(); ++it) {
        Vec2 p = *it;
        while (upper.size() >= 2 && cross(upper.back() - upper[upper.size() - 2], p - upper.back()) <= eps) {
            upper.pop_back();
        }
        upper.push_back(p);
    }
    lower.pop_back();
    upper.pop_back();
    lower.insert(lower.end(), upper.begin(), upper.end());
    return lower;
}

Ring ringFromPoints(std::vector<Vec2> points, bool isHole = false) {
    Ring ring;
    ring.points = std::move(points);
    ring.isHole = isHole;
    ring.winding = polygonSignedArea(ring.points) >= 0.0 ? 1 : -1;
    return ring;
}

} // namespace

std::vector<Vec2> cleanPolygonPoints(const std::vector<Vec2>& points, double eps) {
    std::vector<Vec2> out;
    out.reserve(points.size());
    for (Vec2 p : points) {
        if (out.empty() || distance(out.back(), p) > eps) {
            out.push_back(p);
        }
    }
    if (out.size() > 1 && distance(out.front(), out.back()) <= eps) {
        out.pop_back();
    }
    while (out.size() >= 3) {
        bool removed = false;
        for (size_t i = 0; i < out.size(); ++i) {
            const Vec2 a = out[(i + out.size() - 1) % out.size()];
            const Vec2 b = out[i];
            const Vec2 c = out[(i + 1) % out.size()];
            if (std::abs(cross(b - a, c - b)) <= eps && dot(b - a, c - b) >= -eps) {
                out.erase(out.begin() + static_cast<std::ptrdiff_t>(i));
                removed = true;
                break;
            }
        }
        if (!removed) {
            break;
        }
    }
    return out;
}

double polygonSignedArea(const std::vector<Vec2>& points) {
    const std::vector<Vec2> clean = cleanPolygonPoints(points);
    if (clean.size() < 3) {
        return 0.0;
    }
    double sum = 0.0;
    for (size_t i = 0; i < clean.size(); ++i) {
        const Vec2 a = clean[i];
        const Vec2 b = clean[(i + 1) % clean.size()];
        sum += a.x * b.y - b.x * a.y;
    }
    return sum * 0.5;
}

bool isConvexPolygon(const std::vector<Vec2>& points, double eps) {
    const std::vector<Vec2> clean = cleanPolygonPoints(points, eps);
    if (clean.size() < 3) {
        return false;
    }
    const double orientation = polygonSignedArea(clean) >= 0.0 ? 1.0 : -1.0;
    for (size_t i = 0; i < clean.size(); ++i) {
        const Vec2 a = clean[i];
        const Vec2 b = clean[(i + 1) % clean.size()];
        const Vec2 c = clean[(i + 2) % clean.size()];
        if (orientation * cross(b - a, c - b) < -eps) {
            return false;
        }
    }
    return true;
}

Ring convexMinkowskiSum(const Ring& a, const Ring& b, double eps) {
    std::vector<Vec2> sums;
    const std::vector<Vec2> pa = cleanPolygonPoints(a.points, eps);
    const std::vector<Vec2> pb = cleanPolygonPoints(b.points, eps);
    sums.reserve(pa.size() * pb.size());
    for (Vec2 va : pa) {
        for (Vec2 vb : pb) {
            sums.push_back(va + vb);
        }
    }
    return ringFromPoints(convexHull(std::move(sums), eps));
}

std::vector<Ring> triangulateSimplePolygon(const Ring& ring, double eps) {
    std::vector<Vec2> points = cleanPolygonPoints(ring.points, eps);
    std::vector<Ring> triangles;
    if (points.size() < 3) {
        return triangles;
    }
    if (polygonSignedArea(points) < 0.0) {
        std::reverse(points.begin(), points.end());
    }
    std::vector<size_t> indices(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        indices[i] = i;
    }
    size_t guard = 0;
    while (indices.size() > 3 && guard++ < points.size() * points.size()) {
        bool clipped = false;
        for (size_t i = 0; i < indices.size(); ++i) {
            const size_t ip = indices[(i + indices.size() - 1) % indices.size()];
            const size_t ic = indices[i];
            const size_t in = indices[(i + 1) % indices.size()];
            const Vec2 a = points[ip];
            const Vec2 b = points[ic];
            const Vec2 c = points[in];
            if (cross(b - a, c - b) <= eps) {
                continue;
            }
            bool contains = false;
            for (size_t index : indices) {
                if (index == ip || index == ic || index == in) {
                    continue;
                }
                if (pointInTriangle(points[index], a, b, c, eps)) {
                    contains = true;
                    break;
                }
            }
            if (contains) {
                continue;
            }
            triangles.push_back(ringFromPoints({a, b, c}));
            indices.erase(indices.begin() + static_cast<std::ptrdiff_t>(i));
            clipped = true;
            break;
        }
        if (!clipped) {
            break;
        }
    }
    if (indices.size() == 3) {
        triangles.push_back(ringFromPoints({points[indices[0]], points[indices[1]], points[indices[2]]}));
    }
    if (triangles.empty()) {
        triangles.push_back(ringFromPoints(points));
    }
    return triangles;
}

MinkowskiResult minkowskiSumDecomposed(const Ring& a, const Ring& b, double eps) {
    MinkowskiResult result;
    const std::vector<Ring> partsA = isConvexPolygon(a.points, eps) ? std::vector<Ring>{ringFromPoints(cleanPolygonPoints(a.points, eps), a.isHole)} : triangulateSimplePolygon(a, eps);
    const std::vector<Ring> partsB = isConvexPolygon(b.points, eps) ? std::vector<Ring>{ringFromPoints(cleanPolygonPoints(b.points, eps), b.isHole)} : triangulateSimplePolygon(b, eps);
    result.usedDecomposition = partsA.size() > 1 || partsB.size() > 1;
    for (const Ring& pa : partsA) {
        for (const Ring& pb : partsB) {
            Ring sum = convexMinkowskiSum(pa, pb, eps);
            if (sum.points.size() >= 3) {
                result.components.push_back({std::move(sum), !result.usedDecomposition});
            }
        }
    }
    return result;
}

Ring reflectedRing(const Ring& ring) {
    Ring out = ring;
    for (Vec2& p : out.points) {
        p.x = -p.x;
        p.y = -p.y;
    }
    std::reverse(out.points.begin(), out.points.end());
    out.winding = polygonSignedArea(out.points) >= 0.0 ? 1 : -1;
    return out;
}

} // namespace nest
