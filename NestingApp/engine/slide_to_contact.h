#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_state.h"

namespace nest {

struct SlideToContactResult {
    Pose pose;
    bool moved = false;
    double distance = 0.0;
};

class SlideToContact {
public:
    SlideToContactResult slide(
        const Document& document,
        const EngineSettings& settings,
        const LayoutState& state,
        size_t partIndex,
        const Pose& initialPose,
        Vec2 direction,
        double maxDistance) const;
};

} // namespace nest
