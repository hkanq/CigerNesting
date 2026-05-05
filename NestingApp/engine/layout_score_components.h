#pragma once

#include "core/document.h"
#include "core/polygon.h"
#include "engine/engine_settings.h"
#include <vector>

namespace nest {

struct LayoutShapeMetrics {
    double towerScore = 0.0;
    double layoutSpreadScore = 0.0;
    double unusedSheetRegionScore = 0.0;
};

double cavityPlacementReward(const Document& document, const std::vector<Pose>& poses);
LayoutShapeMetrics computeLayoutShapeMetrics(const Document& document, const EngineSettings& settings, const AABB& usedBounds);

} // namespace nest
