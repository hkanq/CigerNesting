#include "engine/parallel_collision_evaluator.h"

#include <algorithm>
#include <future>
#include <thread>

namespace nest {
namespace {

CollisionReport evaluatePairRange(
    const std::vector<Part>& parts,
    const std::vector<Pose>& poses,
    const std::vector<std::pair<size_t, size_t>>& pairs,
    size_t begin,
    size_t end,
    double tolerance) {
    CollisionReport report;
    for (size_t i = begin; i < end; ++i) {
        const auto& [a, b] = pairs[i];
        if (a >= parts.size() || b >= parts.size() || a >= poses.size() || b >= poses.size()) {
            continue;
        }
        const AABB boxA = transformedBounds(parts[a], poses[a]);
        const AABB boxB = transformedBounds(parts[b], poses[b]);
        const double aabbScore = aabbOverlapArea(boxA, boxB);
        if (aabbScore <= 0.0) {
            continue;
        }
        if (partsCollide(parts[a], poses[a], parts[b], poses[b], tolerance)) {
            ++report.collisionCount;
            report.overlapScore += aabbScore;
            report.pairs.push_back({a, b});
        }
    }
    return report;
}

} // namespace

size_t ParallelCollisionEvaluator::resolveThreadCount(const EngineSettings& settings) {
    const unsigned int hardwareRaw = std::thread::hardware_concurrency();
    const size_t hardware = hardwareRaw == 0 ? 1 : static_cast<size_t>(hardwareRaw);

    if (settings.cpuThreadCount > 0) {
        return std::max<size_t>(1, static_cast<size_t>(settings.cpuThreadCount));
    }

    if (settings.performanceProfile == PerformanceProfile::Maximum) {
        return hardware;
    }

    if (settings.performanceProfile == PerformanceProfile::Fast) {
        return std::max<size_t>(1, hardware / 2);
    }

    return hardware > 1 ? hardware - 1 : 1;
}

CollisionReport ParallelCollisionEvaluator::evaluate(
    const std::vector<Part>& parts,
    const std::vector<Pose>& poses,
    const std::vector<std::pair<size_t, size_t>>& pairs,
    double tolerance,
    WorkerPool& workerPool,
    size_t threadCount) const {
    CollisionReport combined;
    if (pairs.empty()) {
        return combined;
    }

    const size_t taskCount = std::max<size_t>(1, std::min(threadCount, pairs.size()));
    const size_t chunkSize = (pairs.size() + taskCount - 1) / taskCount;
    std::vector<std::future<CollisionReport>> futures;
    futures.reserve(taskCount);

    for (size_t task = 0; task < taskCount; ++task) {
        const size_t begin = task * chunkSize;
        const size_t end = std::min(pairs.size(), begin + chunkSize);
        if (begin >= end) {
            break;
        }
        futures.push_back(workerPool.enqueue([&parts, &poses, &pairs, begin, end, tolerance]() {
            return evaluatePairRange(parts, poses, pairs, begin, end, tolerance);
        }));
    }

    for (auto& future : futures) {
        CollisionReport local = future.get();
        combined.collisionCount += local.collisionCount;
        combined.overlapScore += local.overlapScore;
        combined.pairs.insert(combined.pairs.end(), local.pairs.begin(), local.pairs.end());
    }

    return combined;
}

} // namespace nest
