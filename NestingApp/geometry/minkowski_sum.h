#pragma once

#include "core/polygon.h"
#include <vector>

namespace nest {

struct MinkowskiComponent {
    Ring ring;
    bool exactConvex = false;
};

struct MinkowskiResult {
    std::vector<MinkowskiComponent> components;
    bool usedDecomposition = false;
};

std::vector<Vec2> cleanPolygonPoints(const std::vector<Vec2>& points, double eps = 1e-7);
double polygonSignedArea(const std::vector<Vec2>& points);
bool isConvexPolygon(const std::vector<Vec2>& points, double eps = 1e-9);
Ring convexMinkowskiSum(const Ring& a, const Ring& b, double eps = 1e-7);
std::vector<Ring> triangulateSimplePolygon(const Ring& ring, double eps = 1e-7);
MinkowskiResult minkowskiSumDecomposed(const Ring& a, const Ring& b, double eps = 1e-7);
Ring reflectedRing(const Ring& ring);

} // namespace nest
