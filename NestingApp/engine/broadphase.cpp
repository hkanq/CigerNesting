#include "engine/broadphase.h"

#include "geometry/spatial_grid.h"
#include <algorithm>

namespace nest {

std::vector<std::pair<size_t, size_t>> BroadPhase::findCandidatePairs(const std::vector<Part>& parts, const std::vector<Pose>& poses, double spacing) const {
    SpatialGrid grid(std::max(32.0, spacing * 8.0 + 64.0));
    const size_t count = std::min(parts.size(), poses.size());
    for (size_t i = 0; i < count; ++i) {
        grid.insert(i, transformedBounds(parts[i], poses[i]).expanded(spacing));
    }
    return grid.candidatePairs();
}

} // namespace nest
