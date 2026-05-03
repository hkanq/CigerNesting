#pragma once

#include "core/part.h"
#include "geometry/collision.h"
#include <vector>

namespace nest {

class NarrowPhase {
public:
    CollisionReport evaluate(const std::vector<Part>& parts, const std::vector<Pose>& poses, const std::vector<std::pair<size_t, size_t>>& pairs, double tolerance) const;
};

} // namespace nest
