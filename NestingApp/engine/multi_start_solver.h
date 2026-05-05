#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_state.h"
#include "engine/penalty_system.h"
#include "engine/solver_state.h"
#include <atomic>
#include <chrono>
#include <functional>

namespace nest {

struct SolverProgress {
    SolverPhase phase = SolverPhase::Idle;
    SolverStrategy currentStrategy = SolverStrategy::Idle;
    double progress = 0.0;
    LayoutState current;
    LayoutState best;
    double elapsedSeconds = 0.0;
    SolverStats stats;
    ActiveMoveSummary activeMoves;
    uint64_t versionId = 0;
    bool layoutChanged = false;
    size_t lastMovedPart = kNoPartIndex;
    SolverStrategy lastMoveStrategy = SolverStrategy::Idle;
    bool bestUpdated = false;
};

using SolverProgressCallback = std::function<void(const SolverProgress&)>;

class MultiStartSolver {
public:
    LayoutState solve(
        const Document& document,
        const EngineSettings& settings,
        const std::atomic_bool& stopRequested,
        SolverProgressCallback callback) const;

    LayoutState rowBaseline(const Document& document, const EngineSettings& settings, PlacementStrategy strategy, unsigned int seed, int orderMode) const;
    LayoutState contourSeedBaseline(const Document& document, const EngineSettings& settings, unsigned int seed) const;
    SolverStats lastStats() const { return lastStats_; }

private:
    std::vector<PlacementStrategy> strategySchedule(PlacementStrategy preferred) const;
    std::vector<size_t> partOrder(const Document& document, unsigned int seed, int orderMode) const;
    mutable SolverStats lastStats_{};
};

} // namespace nest
