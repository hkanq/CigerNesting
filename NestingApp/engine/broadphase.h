#pragma once

#include "core/part.h"
#include <cstddef>
#include <utility>
#include <vector>

namespace nest {

class BroadPhase {
public:
    std::vector<std::pair<size_t, size_t>> findCandidatePairs(const std::vector<Part>& parts, const std::vector<Pose>& poses, double spacing) const;
};

} // namespace nest
