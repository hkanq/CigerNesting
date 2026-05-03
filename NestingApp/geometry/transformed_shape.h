#pragma once

#include "core/aabb.h"
#include "core/part.h"
#include <vector>

namespace nest {

struct TransformedRing {
    std::vector<Vec2> points;
    bool isHole = false;
    AABB bounds;
};

struct TransformedPart {
    int partId = -1;
    std::vector<TransformedRing> rings;
    AABB bounds;
};

TransformedPart transformPart(const Part& part, const Pose& pose, int partId = -1);

} // namespace nest
