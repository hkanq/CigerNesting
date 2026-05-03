#include "engine/local_search.h"

#include "engine/broadphase.h"
#include "engine/narrowphase.h"

namespace nest {

void LocalSearch::resolveSimpleCollisions(const Document& document, const EngineSettings& settings, std::vector<Pose>& poses) const {
    BroadPhase broad;
    NarrowPhase narrow;
    for (int iteration = 0; iteration < 8; ++iteration) {
        const auto pairs = broad.findCandidatePairs(document.parts, poses, settings.partSpacing);
        const auto report = narrow.evaluate(document.parts, poses, pairs, settings.collisionTolerance);
        if (report.collisionCount == 0) {
            return;
        }
        for (const auto& pair : report.pairs) {
            if (pair.b < poses.size()) {
                poses[pair.b].x += settings.partSpacing + 2.0 + iteration;
                poses[pair.b].y += 0.5;
            }
        }
    }
}

} // namespace nest
