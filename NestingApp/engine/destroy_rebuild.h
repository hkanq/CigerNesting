#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_state.h"
#include "engine/penalty_system.h"
#include "engine/solver_state.h"
#include <atomic>

namespace nest {

class DestroyRebuild {
public:
    LayoutState improve(
        const Document& document,
        const EngineSettings& settings,
        LayoutState current,
        const std::atomic_bool& stopRequested,
        SolverStats* stats = nullptr) const;
};

} // namespace nest
