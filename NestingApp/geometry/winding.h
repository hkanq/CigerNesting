#pragma once

#include "core/vec2.h"
#include <vector>

namespace nest {

double signedArea(const std::vector<Vec2>& points);
int windingDirection(const std::vector<Vec2>& points);
bool isPointInRing(const Vec2& p, const std::vector<Vec2>& ring);

} // namespace nest
