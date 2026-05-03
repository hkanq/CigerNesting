#include "engine/narrowphase.h"

namespace nest {

CollisionReport NarrowPhase::evaluate(const std::vector<Part>& parts, const std::vector<Pose>& poses, const std::vector<std::pair<size_t, size_t>>& pairs, double tolerance) const {
    CollisionReport report;
    for (const auto& [a, b] : pairs) {
        if (a >= parts.size() || b >= parts.size() || a >= poses.size() || b >= poses.size()) {
            continue;
        }
        const AABB boxA = transformedBounds(parts[a], poses[a]);
        const AABB boxB = transformedBounds(parts[b], poses[b]);
        const double aabbScore = aabbOverlapArea(boxA, boxB);
        if (aabbScore <= 0.0) {
            continue;
        }
        if (partsOverlap(parts[a], poses[a], parts[b], poses[b], tolerance)) {
            ++report.collisionCount;
            report.overlapScore += aabbScore;
            report.pairs.push_back({a, b});
        }
    }
    return report;
}

} // namespace nest
