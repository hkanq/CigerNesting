#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_state.h"
#include "engine/penalty_system.h"
#include "engine/worker_pool.h"
#include <atomic>
#include <cstddef>

namespace nest {

struct UltraRefinementStats {
    double beforeScore = 0.0;
    double afterScore = 0.0;
    double beforeUtilization = 0.0;
    double afterUtilization = 0.0;
    int acceptedMoves = 0;
    double angleStepMinUsedDegrees = 0.0;
    size_t evaluatedCandidates = 0;
    int collisionCount = 0;
    int invalidPartCount = 0;
};

class UltraRefinement {
public:
    LayoutState refine(
        const Document& document,
        const EngineSettings& settings,
        LayoutState state,
        const PenaltySystem& penalties,
        WorkerPool& workerPool,
        const std::atomic_bool& stopRequested) const;

    UltraRefinementStats lastStats() const { return lastStats_; }

private:
    mutable UltraRefinementStats lastStats_;
};

} // namespace nest
