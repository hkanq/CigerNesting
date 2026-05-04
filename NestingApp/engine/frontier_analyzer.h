#pragma once

#include "core/aabb.h"
#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_state.h"
#include <cstddef>
#include <vector>

namespace nest {

enum class FrontierCandidateKind {
    SheetLine,
    BoundsEdge,
    Skyline,
    EmptyCell,
    ContactSnap
};

struct FrontierCandidate {
    Vec2 anchor;
    FrontierCandidateKind kind = FrontierCandidateKind::Skyline;
    size_t sourcePart = static_cast<size_t>(-1);
    double priority = 0.0;
    AABB featureBounds;
};

class FrontierAnalyzer {
public:
    std::vector<FrontierCandidate> analyze(
        const Document& document,
        const EngineSettings& settings,
        const LayoutState& state) const;
};

} // namespace nest
