#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_state.h"
#include "engine/penalty_system.h"
#include <vector>

namespace nest {

class Compression {
public:
    void compressLeftUp(const Document& document, const EngineSettings& settings, std::vector<Pose>& poses) const;
    void compressByStrategy(const Document& document, const EngineSettings& settings, std::vector<Pose>& poses) const;
    LayoutState compressByScore(const Document& document, const EngineSettings& settings, LayoutState state, const PenaltySystem& penalties) const;
};

} // namespace nest
