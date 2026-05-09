#pragma once

#include "core/part.h"
#include "core/sheet.h"
#include <vector>

namespace nest {

struct InnerFitPolygonLoop {
    Ring ring;
    AABB bounds;
    bool exactRectangular = false;
};

struct InnerFitPolygonResult {
    std::vector<InnerFitPolygonLoop> loops;
    bool exactRectangular = false;
};

struct InnerFitPolygonOptions {
    double margin = 0.0;
    double tolerance = 1e-6;
};

InnerFitPolygonResult buildInnerFitPolygon(
    const Part& moving,
    const Pose& movingOrientation,
    const Sheet& sheet,
    const InnerFitPolygonOptions& options);

std::vector<Pose> sampleInnerFitPolygonCandidates(
    const InnerFitPolygonResult& ifp,
    const Pose& orientation,
    size_t limit,
    Vec2 target = {});

} // namespace nest
