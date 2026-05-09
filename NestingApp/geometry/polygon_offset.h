#pragma once

#include "core/polygon.h"
#include <vector>

namespace nest {

enum class OffsetJoinStyle {
    Miter,
    Bevel,
    Round
};

struct PolygonOffsetOptions {
    double distance = 0.0;
    double tolerance = 1e-6;
    OffsetJoinStyle joinStyle = OffsetJoinStyle::Miter;
    double miterLimit = 4.0;
};

Ring offsetRingConservative(const Ring& ring, const PolygonOffsetOptions& options);
std::vector<Ring> offsetRingsConservative(const std::vector<Ring>& rings, const PolygonOffsetOptions& options);

} // namespace nest
