#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_state.h"
#include "engine/penalty_system.h"
#include "engine/worker_pool.h"
#include <atomic>

namespace nest {

class OverlapResolver {
public:
    LayoutState resolve(
        const Document& document,
        const EngineSettings& settings,
        LayoutState state,
        PenaltySystem& penalties,
        WorkerPool& workerPool,
        const std::atomic_bool& stopRequested,
        unsigned int seed) const;
};

} // namespace nest
