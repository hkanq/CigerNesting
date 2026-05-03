#pragma once

#include "core/polygon.h"
#include "geometry/collision.h"
#include <cstddef>
#include <vector>

namespace nest {

struct LayoutState {
    std::vector<Pose> poses;
    double usedWidth = 0.0;
    double usedHeight = 0.0;
    double utilization = 0.0;
    int collisionCount = 0;
    int invalidPartCount = 0;
    double overlapPenalty = 0.0;
    double spacingPenalty = 0.0;
    double sheetPenalty = 0.0;
    double totalScore = 0.0;
    std::vector<CollisionPair> collisionPairs;

    bool valid() const {
        return collisionCount == 0 && invalidPartCount == 0 && spacingPenalty <= 0.0 && sheetPenalty <= 0.0;
    }
};

} // namespace nest
