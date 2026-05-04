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
    unsigned int seed,
    SolverStats* stats) const {
    GuidedLocalSearch search;
    int iterations = 18;
    if (settings.performanceProfile == PerformanceProfile::Fast) {
        iterations = 8;
    } else if (settings.performanceProfile == PerformanceProfile::Maximum) {
        iterations = 28;
    } else if (settings.qualityMode == QualityMode::Fast) {
        iterations = 10;
    }
    if (document.parts.size() > 300) {
        iterations = std::max(2, iterations / 3);
    } else if (document.parts.size() > 100) {
        iterations = std::max(3, iterations / 2);
    }
    return search.improve(document, settings, std::move(state), attemptPenalties, workerPool, stopRequested, seed, iterations, stats);
}

} // namespace nest
