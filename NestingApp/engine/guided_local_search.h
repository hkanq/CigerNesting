#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_score.h"
#include "engine/layout_state.h"
#include "engine/penalty_system.h"
#include "engine/pose_sampler.h"
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
        int maxIterations) const;

private:
    bool tryImprovePart(
        const Document& document,
        const EngineSettings& settings,
        LayoutState& state,
        const PenaltySystem& attemptPenalties,
        WorkerPool& workerPool,
        size_t partIndex,
        unsigned int seed,
        int iteration) const;
};

} // namespace nest
