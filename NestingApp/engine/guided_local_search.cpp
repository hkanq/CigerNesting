#include "engine/guided_local_search.h"

#include <algorithm>
#include <future>
#include <mutex>
#include <vector>

namespace nest {
namespace {

void mergeStats(SolverStats& target, const SolverStats& source) {
    target.evaluatedCandidates += source.evaluatedCandidates;
    target.acceptedMoves += source.acceptedMoves;
    target.rejectedCollision += source.rejectedCollision;
    target.rejectedSpacing += source.rejectedSpacing;
    target.rejectedSheet += source.rejectedSheet;
    target.cacheHits += source.cacheHits;
    target.cacheMisses += source.cacheMisses;
}

void classifyCandidate(const DeltaEvaluation& trial, SolverStats& stats) {
    if (trial.collisionCount > 0) {
        ++stats.rejectedCollision;
    } else if (trial.spacingPenalty > 0.0) {
        ++stats.rejectedSpacing;
    } else if (trial.invalidPartCount > 0 || trial.sheetPenalty > 0.0) {
        ++stats.rejectedSheet;
    }
}

struct CandidateBatchResult {
    bool found = false;
    Pose bestPose;
    double bestScore = 0.0;
    SolverStats stats;
};

} // namespace

LayoutState GuidedLocalSearch::improve(
    const Document& document,
    const EngineSettings& settings,
    LayoutState state,
    PenaltySystem& attemptPenalties,
    WorkerPool& workerPool,
    const std::atomic_bool& stopRequested,
    unsigned int seed,
    int maxIterations,
    SolverStats* stats) const {
    LayoutScore scorer;
    state = scorer.evaluate(document, settings, state.poses, &attemptPenalties);
    CandidateCache cache(settings.performanceProfile == PerformanceProfile::Maximum ? 16384 : settings.performanceProfile == PerformanceProfile::Balanced ? 8192 : 4096);

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
            changed = tryImprovePart(document, settings, state, attemptPenalties, cache, workerPool, stopRequested, partIndex, seed, iteration, stats) || changed;
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
    CandidateCache& cache,
    WorkerPool& workerPool,
    const std::atomic_bool& stopRequested,
    size_t partIndex,
    unsigned int seed,
    int iteration,
    SolverStats* stats) const {
    if (partIndex >= state.poses.size() || partIndex >= document.parts.size()) {
        return false;
    }

    PoseSampler sampler;
    LayoutScore scorer;
    LayoutEvalCache evalCache;
    evalCache.rebuild(document, settings, state, &attemptPenalties);
    Pose bestPose = state.poses[partIndex];
    double bestScore = state.totalScore;
    LayoutState best = state;
    const auto candidates = sampler.moveCandidates(document, settings, state.poses, partIndex, seed, iteration);
    const Pose basePose = state.poses[partIndex];
    std::mutex cacheMutex;

    const size_t workerCount = std::max<size_t>(1, std::min(workerPool.threadCount(), candidates.size()));
    const size_t chunkSize = (candidates.size() + workerCount - 1) / workerCount;
    std::vector<std::future<CandidateBatchResult>> futures;
    futures.reserve(workerCount);

    for (size_t worker = 0; worker < workerCount; ++worker) {
        const size_t begin = worker * chunkSize;
        const size_t end = std::min(candidates.size(), begin + chunkSize);
        if (begin >= end) {
            break;
        }
        futures.push_back(workerPool.enqueue([&, begin, end]() {
            CandidateBatchResult result{false, basePose, state.totalScore, {}};
            for (size_t i = begin; i < end && !stopRequested.load(); ++i) {
                CachedCandidateScore cached;
                bool cacheHit = false;
                {
                    std::lock_guard<std::mutex> lock(cacheMutex);
                    cacheHit = cache.get(partIndex, basePose, candidates[i], cached);
                }

                if (cacheHit) {
                    ++result.stats.cacheHits;
                    if (cached.totalScore + 1e-9 >= result.bestScore) {
                        continue;
                    }
                } else {
                    ++result.stats.cacheMisses;
                }

                const DeltaMove move{partIndex, basePose, candidates[i]};
                const DeltaEvaluation trial = evaluateMoveDelta(document, settings, state, evalCache, move);
                ++result.stats.evaluatedCandidates;
                classifyCandidate(trial, result.stats);
                if (!cacheHit) {
                    const CachedCandidateScore score{
                        trial.totalScore,
                        trial.valid,
                        trial.collisionCount,
                        trial.invalidPartCount,
                        trial.spacingPenalty,
                        trial.sheetPenalty
                    };
                    std::lock_guard<std::mutex> lock(cacheMutex);
                    cache.put(partIndex, basePose, candidates[i], score);
                }
                if (trial.totalScore + 1e-9 < result.bestScore) {
                    result.found = true;
                    result.bestPose = candidates[i];
                    result.bestScore = trial.totalScore;
                }
            }
            return result;
        }));
    }

    for (auto& future : futures) {
        CandidateBatchResult trial = future.get();
        if (stats) {
            mergeStats(*stats, trial.stats);
        }
        if (trial.found && trial.bestScore + 1e-9 < bestScore) {
            bestPose = trial.bestPose;
            bestScore = trial.bestScore;
        }
    }

    if (bestScore + 1e-9 < state.totalScore) {
        std::vector<Pose> trialPoses = state.poses;
        trialPoses[partIndex] = bestPose;
        best = scorer.evaluate(document, settings, trialPoses, &attemptPenalties);
    }

    if (best.totalScore + 1e-9 < state.totalScore) {
        state = std::move(best);
        if (stats) {
            ++stats->acceptedMoves;
        }
        return true;
    }
    return false;
}

} // namespace nest
