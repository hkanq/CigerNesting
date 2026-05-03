#pragma once

#include "core/polygon.h"
#include <string>
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

inline const char* phaseName(SolverPhase phase) {
    switch (phase) {
    case SolverPhase::Idle: return "Idle";
    case SolverPhase::PrepareGeometry: return "PrepareGeometry";
    case SolverPhase::InitialPlacement: return "InitialPlacement";
    case SolverPhase::Exploration: return "Exploration";
    case SolverPhase::CollisionResolution: return "CollisionResolution";
    case SolverPhase::Compression: return "Compression";
    case SolverPhase::UltraRefinement: return "UltraRefinement";
    case SolverPhase::FinalValidation: return "FinalValidation";
    case SolverPhase::Done: return "Done";
    case SolverPhase::Stopped: return "Stopped";
    }
    return "Unknown";
}

struct SolverSnapshot {
    std::vector<Pose> currentPoses;
    std::vector<Pose> bestPoses;
    std::string phaseName;
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
