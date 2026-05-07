#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_state.h"
#include "engine/penalty_system.h"
#include "engine/solver_state.h"
#include <atomic>
#include <vector>

namespace nest {

// Legacy local compaction helper. In Balanced/Maximum it is a micro-correction tool, not the main placement strategy.
class Compression {
public:
    void compressLeftUp(const Document& document, const EngineSettings& settings, std::vector<Pose>& poses) const;
    void compressByStrategy(const Document& document, const EngineSettings& settings, std::vector<Pose>& poses) const;
    LayoutState compressByScore(
        const Document& document,
        const EngineSettings& settings,
        LayoutState state,
        const PenaltySystem& penalties,
        SolverStats* stats = nullptr,
        const std::atomic_bool* stopRequested = nullptr) const;
};

} // namespace nest
