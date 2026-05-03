#include "engine/guided_local_search.h"

#include <algorithm>
#include <future>
#include <vector>

namespace nest {

LayoutState GuidedLocalSearch::improve(
    const Document& document,
    const EngineSettings& settings,
    LayoutState state,
    PenaltySystem& attemptPenalties,
    WorkerPool& workerPool,
    const std::atomic_bool& stopRequested,
    unsigned int seed,
    int maxIterations) const {
    LayoutScore scorer;
    state = scorer.evaluate(document, settings, state.poses, &attemptPenalties);

    for (int iteration = 0; iteration < maxIterations && !stopRequested.load(); ++iteration) {
        const LayoutState before = state;
        for (const CollisionPair& pair : state.collisionPairs) {
            attemptPenalties.observeCollision(pair.a, pair.b);
        }

        std::vector<size_t> targets;
        targets.reserve(state.collisionPairs.size() * 2 + document.parts.size());
        for (const CollisionPair& pair : state.collisionPairs) {
            targets.push_back(pair.b);
            targets.push_back(pair.a);
        }
        if (targets.empty()) {
            for (size_t i = 0; i < document.parts.size(); ++i) {
                targets.push_back((i + static_cast<size_t>(iteration)) % document.parts.size());
            }
        }

        bool changed = false;
        for (size_t partIndex : targets) {
            if (stopRequested.load()) {
                break;
            }
            changed = tryImprovePart(document, settings, state, attemptPenalties, workerPool, partIndex, seed, iteration) || changed;
            if (state.valid()) {
                break;
            }
        }

        state = scorer.evaluate(document, settings, state.poses, &attemptPenalties);
        if (!changed && state.totalScore >= before.totalScore - 1e-9) {
            break;
        }
    }

    return state;
}

bool GuidedLocalSearch::tryImprovePart(
    const Document& document,
    const EngineSettings& settings,
    LayoutState& state,
    const PenaltySystem& attemptPenalties,
    WorkerPool& workerPool,
    size_t partIndex,
    unsigned int seed,
    int iteration) const {
    if (partIndex >= state.poses.size() || partIndex >= document.parts.size()) {
        return false;
    }

    PoseSampler sampler;
    LayoutScore scorer;
    LayoutState best = state;
    const auto candidates = sampler.moveCandidates(document, settings, state.poses, partIndex, seed, iteration);

    const size_t workerCount = std::max<size_t>(1, std::min(workerPool.threadCount(), candidates.size()));
    const size_t chunkSize = (candidates.size() + workerCount - 1) / workerCount;
    std::vector<std::future<LayoutState>> futures;
    futures.reserve(workerCount);

    for (size_t worker = 0; worker < workerCount; ++worker) {
        const size_t begin = worker * chunkSize;
        const size_t end = std::min(candidates.size(), begin + chunkSize);
        if (begin >= end) {
            break;
        }
        futures.push_back(workerPool.enqueue([&, begin, end]() {
            LayoutState localBest = state;
            for (size_t i = begin; i < end; ++i) {
                std::vector<Pose> trialPoses = state.poses;
                trialPoses[partIndex] = candidates[i];
                LayoutState trial = scorer.evaluate(document, settings, trialPoses, &attemptPenalties);
                if (trial.totalScore + 1e-9 < localBest.totalScore) {
                    localBest = std::move(trial);
                }
            }
            return localBest;
        }));
    }

    for (auto& future : futures) {
        LayoutState trial = future.get();
        if (trial.totalScore + 1e-9 < best.totalScore) {
            best = std::move(trial);
        }
    }

    if (best.totalScore + 1e-9 < state.totalScore) {
        state = std::move(best);
        return true;
    }
    return false;
}

} // namespace nest
