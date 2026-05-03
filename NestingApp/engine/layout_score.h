#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_state.h"
#include "engine/penalty_system.h"

namespace nest {

class LayoutScore {
public:
    LayoutState evaluate(
        const Document& document,
        const EngineSettings& settings,
        const std::vector<Pose>& poses,
        const PenaltySystem* attemptPenalties = nullptr,
        const PenaltySystem* globalPenalties = nullptr,
        double globalPenaltyWeight = 0.10) const;
};

} // namespace nest
