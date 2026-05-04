#include "engine/guided_local_search.h"

#include "engine/acceptance.h"
#include <algorithm>
#include <future>
#include <limits>
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
    target.swapAttempts += source.swapAttempts;
    target.swapAccepted += source.swapAccepted;
    target.chainAttempts += source.chainAttempts;
    target.chainAccepted += source.chainAccepted;
    target.clusterAttempts += source.clusterAttempts;
    target.clusterAccepted += source.clusterAccepted;
    target.acceptedWorseMoves += source.acceptedWorseMoves;
    target.rejectedWorseMoves += source.rejectedWorseMoves;
    target.tabuRejected += source.tabuRejected;
    target.escapeAttempts += source.escapeAttempts;
    target.escapeAccepted += source.escapeAccepted;
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
    bool acceptedWorse = false;
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
    TabuMemory tabu(settings.performanceProfile == PerformanceProfile::Maximum ? 1024 : settings.performanceProfile == PerformanceProfile::Balanced ? 512 : 192);
    tabu.rememberLayout(state.poses);

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
            changed = tryImprovePart(document, settings, state, attemptPenalties, cache, workerPool, stopRequested, partIndex, seed, iteration, maxIterations, tabu, stats) || changed;
            if (state.valid()) {
                break;
            }
        }

        state = scorer.evaluate(document, settings, state.poses, &attemptPenalties);
        if (!changed && state.totalScore >= before.totalScore - 1e-9) {
            for (const CollisionPair& pair : state.collisionPairs) {
                attemptPenalties.observeStalledPair(pair.a, pair.b);
            }
            break;
        }
        for (const CollisionPair& pair : before.collisionPairs) {
            if (std::find_if(state.collisionPairs.begin(), state.collisionPairs.end(), [&](const CollisionPair& current) {
                return (current.a == pair.a && current.b == pair.b) || (current.a == pair.b && current.b == pair.a);
            }) == state.collisionPairs.end()) {
                attemptPenalties.observeResolved(pair.a, pair.b);
            }
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
    int maxIterations,
    TabuMemory& tabu,
    SolverStats* stats) const {
    if (partIndex >= state.poses.size() || partIndex >= document.parts.size()) {
        return false;
    }

    PoseSampler sampler;
    LayoutScore scorer;
    LayoutEvalCache evalCache;
    evalCache.rebuild(document, settings, state, &attemptPenalties);
    Pose bestPose = state.poses[partIndex];
    double bestScore = std::numeric_limits<double>::max();
    bool acceptedWorse = false;
    LayoutState best = state;
    const auto candidates = sampler.moveCandidates(document, settings, state.poses, partIndex, seed, iteration);
    const Pose basePose = state.poses[partIndex];
    std::mutex cacheMutex;
    AcceptanceCriteria acceptance(settings);

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
            CandidateBatchResult result{false, basePose, std::numeric_limits<double>::max(), false, {}};
            for (size_t i = begin; i < end && !stopRequested.load(); ++i) {
                if (tabu.containsMove(partIndex, basePose, candidates[i])) {
                    ++result.stats.tabuRejected;
                    continue;
                }
                CachedCandidateScore cached;
                bool cacheHit = false;
                {
                    std::lock_guard<std::mutex> lock(cacheMutex);
                    cacheHit = cache.get(partIndex, basePose, candidates[i], cached);
                }

                if (cacheHit) {
                    ++result.stats.cacheHits;
                } else {
                    ++result.stats.cacheMisses;
                }

                DeltaEvaluation trial;
                if (cacheHit) {
                    trial.valid = cached.valid;
                    trial.totalScore = cached.totalScore;
                    trial.collisionCount = cached.collisionCount;
                    trial.invalidPartCount = cached.invalidPartCount;
                    trial.spacingPenalty = cached.spacingPenalty;
                    trial.sheetPenalty = cached.sheetPenalty;
                } else {
                    const DeltaMove move{partIndex, basePose, candidates[i]};
                    trial = evaluateMoveDelta(document, settings, state, evalCache, move);
                }
                ++result.stats.evaluatedCandidates;
                if (!trial.valid) {
                    classifyCandidate(trial, result.stats);
                }
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

                if (!trial.valid) {
                    continue;
                }

                const AcceptanceDecision decision = acceptance.decide(
                    state.totalScore,
                    trial.totalScore,
                    iteration,
                    maxIterations,
                    seed + static_cast<unsigned int>(partIndex * 7919u),
                    i);
                if (!decision.accepted) {
                    if (trial.totalScore > state.totalScore) {
                        ++result.stats.rejectedWorseMoves;
                    }
                    continue;
                }

                if (trial.totalScore + 1e-9 < result.bestScore) {
                    result.found = true;
                    result.bestPose = candidates[i];
                    result.bestScore = trial.totalScore;
                    result.acceptedWorse = decision.acceptedWorse;
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
            acceptedWorse = trial.acceptedWorse;
        }
    }

    if (bestScore < std::numeric_limits<double>::max()) {
        std::vector<Pose> trialPoses = state.poses;
        trialPoses[partIndex] = bestPose;
        if (tabu.containsLayout(trialPoses)) {
            if (stats) {
                ++stats->tabuRejected;
            }
            tabu.rememberMove(partIndex, basePose, bestPose);
            return false;
        }
        best = scorer.evaluate(document, settings, trialPoses, &attemptPenalties);
    }

    if (best.valid() && (best.totalScore + 1e-9 < state.totalScore || acceptedWorse)) {
        const bool isWorse = best.totalScore > state.totalScore + 1e-9;
        state = std::move(best);
        tabu.rememberMove(partIndex, basePose, bestPose);
        tabu.rememberLayout(state.poses);
        if (stats) {
            ++stats->acceptedMoves;
            if (isWorse) {
                ++stats->acceptedWorseMoves;
            }
        }
        return true;
    }
    return false;
}

} // namespace nest
