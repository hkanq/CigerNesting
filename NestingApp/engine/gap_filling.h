#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_state.h"
#include "engine/penalty_system.h"
#include "engine/solver_state.h"
#include <atomic>
#include <cstddef>

namespace nest {

struct GapFillingStats {
    size_t generatedAnchors = 0;
    size_t evaluatedCandidates = 0;
    size_t acceptedMoves = 0;
    size_t holeCandidates = 0;
    size_t concavityCandidates = 0;
};

class GapFilling {
public:
    LayoutState fillGaps(
        const Document& document,
        const EngineSettings& settings,
        LayoutState state,
        const PenaltySystem& penalties,
        const std::atomic_bool& stopRequested,
        SolverStats* stats = nullptr) const;

    GapFillingStats lastStats() const { return lastStats_; }

private:
    mutable GapFillingStats lastStats_{};
};

} // namespace nest
