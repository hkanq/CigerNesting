#pragma once

#include "core/part.h"
#include "core/sheet.h"
#include "geometry/transformed_shape.h"

namespace nest {

struct ClearanceResult {
    bool valid = true;
    double minDistance = 0.0;
    Vec2 closestA;
    Vec2 closestB;
    int ringA = -1;
    int ringB = -1;
};

ClearanceResult minimumBoundaryDistance(
    const TransformedPart& a,
    const TransformedPart& b,
    double requiredSpacing,
    double eps);

ClearanceResult minimumPartToRingDistance(
    const TransformedPart& part,
    const Ring& ring,
    double requiredSpacing,
    double eps);

bool partsRespectClearance(
    const Part& a,
    const Pose& poseA,
    const Part& b,
    const Pose& poseB,
    double requiredSpacing,
    double eps);

bool partRespectsSheetClearance(
    const Part& part,
    const Pose& pose,
    const Sheet& sheet,
    double margin,
    double eps);

} // namespace nest
