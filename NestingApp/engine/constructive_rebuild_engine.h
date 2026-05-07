#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_state.h"
#include "engine/penalty_system.h"
#include "engine/solver_state.h"
#include <atomic>
#include <cstdint>
#include <functional>
#include <vector>

namespace nest {

struct ConstructiveRebuildProgress {
    LayoutState current;
    LayoutState best;
    SolverStats stats;
    ActiveMoveSummary activeMoves;
    uint64_t versionId = 0;
    bool layoutChanged = false;
    bool bestUpdated = false;
    std::vector<size_t> changedParts;
};

using ConstructiveRebuildCallback = std::function<void(const ConstructiveRebuildProgress&)>;

class ConstructiveRebuildEngine {
public:
    LayoutState optimize(
        const Document& document,
        const EngineSettings& settings,
        LayoutState initialValid,
        PenaltySystem& globalPenalties,
        const std::atomic_bool& stopRequested,
        SolverStats& stats,
        ConstructiveRebuildCallback callback = {}) const;
};

} // namespace nest
