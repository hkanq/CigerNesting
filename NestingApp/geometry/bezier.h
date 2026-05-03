#pragma once

#include "core/vec2.h"
#include <vector>

namespace nest {

void flattenQuadraticBezier(const Vec2& p0, const Vec2& p1, const Vec2& p2, double tolerance, std::vector<Vec2>& out);
void flattenCubicBezier(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, double tolerance, std::vector<Vec2>& out);

} // namespace nest
