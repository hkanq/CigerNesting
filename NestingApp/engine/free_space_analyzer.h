#pragma once

#include "core/aabb.h"
#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_state.h"
#include <cstddef>
#include <vector>

namespace nest {

enum class FreeSpaceCandidateKind {
    SheetCorner,
    SheetBoundary,
    PartOuter,
    PartHole,
    Concavity,
    UsedBoundsGap,
    ForbiddenZone
};

struct FreeSpaceCandidate {
    Vec2 anchor;
    FreeSpaceCandidateKind kind = FreeSpaceCandidateKind::UsedBoundsGap;
    size_t sourcePart = static_cast<size_t>(-1);
    int sourceRing = -1;
    double priority = 0.0;
    AABB featureBounds;
};

class FreeSpaceAnalyzer {
public:
    std::vector<FreeSpaceCandidate> analyze(
        const Document& document,
        const EngineSettings& settings,
        const LayoutState& state) const;
};

} // namespace nest
