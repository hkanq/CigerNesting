#pragma once

#include "core/vec2.h"
#include <vector>

namespace nest {

std::vector<Vec2> flattenCircularArc(Vec2 center, double radiusX, double radiusY, double startRadians, double sweepRadians, double tolerance);

} // namespace nest
