#pragma once

#include "core/part.h"
#include "engine/engine_settings.h"
#include "engine/worker_pool.h"
#include "geometry/collision.h"
#include <cstddef>
#include <utility>
#include <vector>

namespace nest {

class ParallelCollisionEvaluator {
public:
    static size_t resolveThreadCount(const EngineSettings& settings);

    CollisionReport evaluate(
        const std::vector<Part>& parts,
        const std::vector<Pose>& poses,
        const std::vector<std::pair<size_t, size_t>>& pairs,
        double tolerance,
        WorkerPool& workerPool,
        size_t threadCount) const;
};

} // namespace nest
