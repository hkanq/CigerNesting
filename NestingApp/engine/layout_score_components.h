#pragma once

#include "core/document.h"
#include "core/polygon.h"
#include <vector>

namespace nest {

double cavityPlacementReward(const Document& document, const std::vector<Pose>& poses);

} // namespace nest
