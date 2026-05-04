#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_state.h"
#include "engine/penalty_system.h"
#include "engine/solver_state.h"
#include <atomic>

namespace nest {

class SmallPartFiller {
public:
    LayoutState fill(
        const Document& document,
        const EngineSettings& settings,
        LayoutState state,
        const PenaltySystem& penalties,
        const std::atomic_bool& stopRequested,
        SolverStats* stats = nullptr) const;
};

} // namespace nest
