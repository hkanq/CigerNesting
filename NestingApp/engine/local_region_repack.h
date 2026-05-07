#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_state.h"
#include "engine/solver_state.h"
#include <atomic>

namespace nest {

// Deprecated main-path module: kept as a diagnostic/test-only helper.
// Production Balanced/Maximum search now routes region rebuild decisions through ConstructiveRebuildEngine.
class LocalRegionRepack {
public:
    LayoutState improve(
        const Document& document,
        const EngineSettings& settings,
        LayoutState state,
        const std::atomic_bool& stopRequested,
        SolverStats* stats = nullptr) const;
};

} // namespace nest
