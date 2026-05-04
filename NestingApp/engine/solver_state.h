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
    ContactPacking,
    GapFilling,
    Rearrangement,
    Escape,
    UltraRefinement,
    FinalValidation,
    Done,
    NoValidLayout,
    Failed,
    Stopped
};

struct SolverStats {
    size_t evaluatedCandidates = 0;
    size_t acceptedMoves = 0;
    size_t rejectedCollision = 0;
    size_t rejectedSpacing = 0;
    size_t rejectedSheet = 0;
    size_t attemptsStarted = 0;
    size_t attemptsCompleted = 0;
    size_t bestUpdates = 0;
    size_t gapAccepted = 0;
    double elapsedMs = 0.0;
    size_t workerCount = 0;
    double candidatesPerSecond = 0.0;
    size_t cacheHits = 0;
    size_t cacheMisses = 0;
    size_t swapAttempts = 0;
    size_t swapAccepted = 0;
    size_t chainAttempts = 0;
    size_t chainAccepted = 0;
    size_t clusterAttempts = 0;
    size_t clusterAccepted = 0;
    size_t acceptedWorseMoves = 0;
    size_t rejectedWorseMoves = 0;
    size_t tabuRejected = 0;
    size_t escapeAttempts = 0;
    size_t escapeAccepted = 0;
    size_t ultraAccepted = 0;
    size_t compactionAccepted = 0;
    size_t frontierCandidates = 0;
    size_t smallFillerAccepted = 0;
    size_t regionRepackAccepted = 0;
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
    size_t validationFailureCount = 0;
    size_t invalidPartCount = 0;
    SolverStats stats;
    bool running = false;
};

struct SolverResult {
    std::vector<Pose> bestPoses;
    size_t collisionCount = 0;
    double overlapScore = 0.0;
    double utilization = 0.0;
    size_t validationFailureCount = 0;
    size_t invalidPartCount = 0;
    SolverStats stats;
    bool valid = false;
};

} // namespace nest
