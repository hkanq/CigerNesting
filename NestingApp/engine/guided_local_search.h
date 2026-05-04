#pragma once

#include "core/document.h"
#include "engine/candidate_cache.h"
#include "engine/engine_settings.h"
#include "engine/layout_score.h"
#include "engine/layout_eval_cache.h"
#include "engine/layout_state.h"
#include "engine/penalty_system.h"
#include "engine/pose_sampler.h"
#include "engine/solver_state.h"
#include "engine/worker_pool.h"
#include <atomic>

namespace nest {

class GuidedLocalSearch {
public:
    LayoutState improve(
        const Document& document,
        const EngineSettings& settings,
        LayoutState state,
        PenaltySystem& attemptPenalties,
        WorkerPool& workerPool,
        const std::atomic_bool& stopRequested,
        unsigned int seed,
        int maxIterations,
        SolverStats* stats = nullptr) const;

private:
    bool tryImprovePart(
        const Document& document,
        const EngineSettings& settings,
        LayoutState& state,
        const PenaltySystem& attemptPenalties,
        CandidateCache& cache,
        WorkerPool& workerPool,
        const std::atomic_bool& stopRequested,
        size_t partIndex,
        unsigned int seed,
        int iteration,
        SolverStats* stats) const;
};

} // namespace nest
