#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_state.h"
#include "engine/penalty_system.h"
#include "engine/solver_state.h"
#include <atomic>

namespace nest {

class EscapeSearch {
public:
    LayoutState escape(
        const Document& document,
        const EngineSettings& settings,
        LayoutState state,
        const PenaltySystem& penalties,
        const std::atomic_bool& stopRequested,
        unsigned int seed,
        SolverStats* stats) const;
};

} // namespace nest
