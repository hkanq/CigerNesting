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
    double progress = 0.0;
    LayoutState current;
    LayoutState best;
    double elapsedSeconds = 0.0;
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

private:
    std::vector<PlacementStrategy> strategySchedule(PlacementStrategy preferred) const;
    std::vector<size_t> partOrder(const Document& document, unsigned int seed, int orderMode) const;
};

} // namespace nest
