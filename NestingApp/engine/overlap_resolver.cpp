#include "engine/overlap_resolver.h"

#include "engine/guided_local_search.h"

namespace nest {

LayoutState OverlapResolver::resolve(
    const Document& document,
    const EngineSettings& settings,
    LayoutState state,
    PenaltySystem& attemptPenalties,
    WorkerPool& workerPool,
    const std::atomic_bool& stopRequested,
    unsigned int seed) const {
    GuidedLocalSearch search;
    return search.improve(document, settings, std::move(state), attemptPenalties, workerPool, stopRequested, seed, settings.qualityMode == QualityMode::Fast ? 8 : 18);
}

} // namespace nest
