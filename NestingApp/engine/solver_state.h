#pragma once

#include "core/polygon.h"
#include <cstddef>
#include <vector>

namespace nest {

enum class SolverPhase {
    Idle,
    PrepareGeometry,
    InitialPlacement,
    Exploration,
    CollisionResolution,
    Compression,
    UltraRefinement,
    FinalValidation,
    Done,
    Stopped
};

struct SolverSnapshot {
    std::vector<Pose> currentPoses;
    std::vector<Pose> bestPoses;
    SolverPhase phase = SolverPhase::Idle;
    double progress = 0.0;
    size_t collisionCount = 0;
    double overlapScore = 0.0;
    double utilization = 0.0;
    double elapsedSeconds = 0.0;
    bool running = false;
};

struct SolverResult {
    std::vector<Pose> bestPoses;
    size_t collisionCount = 0;
    double overlapScore = 0.0;
    double utilization = 0.0;
    bool valid = false;
};

} // namespace nest
