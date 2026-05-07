#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_state.h"
#include "engine/penalty_system.h"
#include "engine/solver_state.h"
#include <atomic>

namespace nest {

// Legacy local rearrangement helper. Multi-part constructive search is centralized in ConstructiveRebuildEngine.
class Rearrangement {
public:
    LayoutState improve(
        const Document& document,
        const EngineSettings& settings,
        LayoutState state,
        const PenaltySystem& penalties,
        const std::atomic_bool& stopRequested,
        SolverStats* stats) const;
};

} // namespace nest
