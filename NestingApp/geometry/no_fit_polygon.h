#pragma once

#include "core/aabb.h"
#include "core/part.h"
#include "geometry/minkowski_sum.h"
#include <vector>

namespace nest {

struct NoFitPolygonLoop {
    Ring ring;
    AABB bounds;
    bool fromHole = false;
    bool exactConvex = false;
};

struct NoFitPolygonResult {
    std::vector<NoFitPolygonLoop> loops;
    bool usedDecomposition = false;
    size_t componentCount = 0;
};

struct NoFitPolygonOptions {
    double spacing = 0.0;
    double tolerance = 1e-6;
    bool includeHoles = true;
};

NoFitPolygonResult buildNoFitPolygon(
    const Part& moving,
    const Pose& movingOrientation,
    const Part& fixed,
    const Pose& fixedOrientation,
    const NoFitPolygonOptions& options);

std::vector<Pose> sampleNoFitPolygonCandidates(
    const NoFitPolygonResult& nfp,
    const Pose& orientation,
    size_t limit,
    Vec2 target = {});

} // namespace nest
