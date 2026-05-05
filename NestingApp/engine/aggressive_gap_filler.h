#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_state.h"
#include "engine/solver_state.h"
#include <atomic>

namespace nest {

class AggressiveGapFiller {
public:
    LayoutState improve(
        const Document& document,
        const EngineSettings& settings,
        LayoutState state,
        const std::atomic_bool& stopRequested,
        SolverStats* stats) const;
};

} // namespace nest
