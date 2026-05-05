#pragma once

#include "core/aabb.h"
#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_state.h"
#include <cstddef>
#include <vector>

namespace nest {

struct EmptyRegion {
    AABB bounds;
    Vec2 center;
    double area = 0.0;
    size_t cellCount = 0;
    bool touchesSheetBoundary = false;
    bool touchesUsedBoundary = false;
};

struct EmptySpaceMap {
    AABB usedBounds;
    double cellWidth = 1.0;
    double cellHeight = 1.0;
    int columns = 0;
    int rows = 0;
    double totalEmptyArea = 0.0;
    double largestRegionArea = 0.0;
    std::vector<EmptyRegion> regions;

    size_t fillableRegionCount(double minArea) const;
};

class EmptySpaceAnalyzer {
public:
    EmptySpaceMap analyze(
        const Document& document,
        const EngineSettings& settings,
        const LayoutState& state,
        int requestedColumns = 0,
        int requestedRows = 0) const;
};

} // namespace nest
