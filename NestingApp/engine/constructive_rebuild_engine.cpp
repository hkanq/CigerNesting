#include "engine/constructive_rebuild_engine.h"

#include "core/math_utils.h"
#include "engine/adaptive_acceptance.h"
#include "engine/empty_space_map.h"
#include "engine/contact_candidate_provider.h"
#include "engine/layout_score.h"
#include "engine/layout_score_components.h"
#include "engine/pose_sampler.h"
#include "geometry/transformed_shape.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>

namespace nest {
namespace {

using Clock = std::chrono::steady_clock;

struct RebuildTarget {
    EmptyRegion region;
    double importance = 0.0;
};

struct BeamNode {
    std::vector<Pose> poses;
    std::vector<size_t> placed;
    double rank = std::numeric_limits<double>::max();
    double contactPriority = 0.0;
};

struct Objective {
    LayoutState state;
    EmptySpaceMap emptyMap;
    double value = std::numeric_limits<double>::max();
};

double elapsedSeconds(Clock::time_point started) {
    return std::chrono::duration<double>(Clock::now() - started).count();
}

double sheetArea(const Document& document, const EngineSettings& settings) {
    const double width = document.sheet.width > 0.0 ? document.sheet.width : settings.sheetWidth;
    const double height = document.sheet.height > 0.0 ? document.sheet.height : settings.sheetHeight;
    return std::max(1.0, (width - settings.margin * 2.0) * (height - settings.margin * 2.0));
}

double partAreaScore(const Part& part) {
    return part.area > 0.0 ? part.area : std::max(1.0, part.localBounds.area());
}

bool containsIndex(const std::vector<size_t>& values, size_t index) {
    return std::find(values.begin(), values.end(), index) != values.end();
}

void appendUnique(std::vector<size_t>& values, size_t index, size_t limit) {
    if (values.size() >= limit || containsIndex(values, index)) {
        return;
    }
    values.push_back(index);
}

bool poseChanged(const Pose& a, const Pose& b) {
    return a.mirrored != b.mirrored ||
        std::abs(a.x - b.x) > 1e-6 ||
        std::abs(a.y - b.y) > 1e-6 ||
        std::abs(a.angleRadians - b.angleRadians) > 1e-8;
}

std::vector<size_t> changedParts(const std::vector<Pose>& a, const std::vector<Pose>& b, const std::vector<size_t>& subset) {
    std::vector<size_t> changed;
    for (size_t index : subset) {
        if (index < a.size() && index < b.size() && poseChanged(a[index], b[index])) {
            changed.push_back(index);
        }
    }
    return changed;
}

bool betterQuality(const LayoutState& candidate, const LayoutState& incumbent, double candidateObjective, double incumbentObjective) {
    if (!candidate.valid()) {
        return false;
    }
    if (!incumbent.valid()) {
        return true;
    }
    if (candidate.utilization + 0.003 < incumbent.utilization) {
        return false;
    }
    if (candidate.utilization > incumbent.utilization + 1e-6) {
        return true;
    }
    if (candidate.usedWidth * candidate.usedHeight + 1.0 < incumbent.usedWidth * incumbent.usedHeight) {
        return true;
    }
    return candidateObjective + 1e-6 < incumbentObjective;
}

Objective evaluateObjective(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    PenaltySystem& attemptPenalties,
    PenaltySystem& globalPenalties) {
    Objective objective;
    LayoutScore scorer;
    objective.state = scorer.evaluate(document, settings, poses, &attemptPenalties, &globalPenalties, 0.10);
    if (!objective.state.valid()) {
        return objective;
    }
    objective.emptyMap = EmptySpaceAnalyzer{}.analyze(document, settings, objective.state);
    const AABB used = objective.emptyMap.usedBounds;
    const double usedArea = std::max(1.0, objective.state.usedWidth * objective.state.usedHeight);
    const double totalArea = std::max(1.0, document.totalPartArea());
    const LayoutShapeMetrics shapeMetrics = computeLayoutShapeMetrics(document, settings, used);
    const double largestGapWeight = settings.performanceProfile == PerformanceProfile::Maximum ? 9.0 : 5.5;
    const double emptyWeight = settings.performanceProfile == PerformanceProfile::Maximum ? 1.9 : 1.2;
    const double boundsWeight = settings.performanceProfile == PerformanceProfile::Maximum ? 2.4 : 1.5;
    const double contactWeight = settings.performanceProfile == PerformanceProfile::Maximum ? 1400.0 : 850.0;
    objective.value = objective.state.totalScore +
        objective.emptyMap.largestRegionArea * largestGapWeight +
        objective.emptyMap.totalEmptyArea * emptyWeight +
        usedArea * boundsWeight +
        shapeMetrics.towerScore * totalArea * 7.0 -
        objective.state.contactReward * contactWeight -
        objective.state.utilization * 160000.0;
    return objective;
}

std::vector<RebuildTarget> makeTargets(const EmptySpaceMap& map, const Document& document, const EngineSettings& settings) {
    std::vector<RebuildTarget> targets;
    const double usableArea = sheetArea(document, settings);
    for (const EmptyRegion& region : map.regions) {
        RebuildTarget target;
        target.region = region;
        const double fillability = std::min(1.0, region.area / std::max(1.0, document.totalPartArea() * 0.08));
        const double boundaryBias = region.touchesUsedBoundary ? 0.35 : 0.0;
        const double sizeBias = region.area / usableArea;
        target.importance = region.area * (1.0 + fillability + boundaryBias + sizeBias * 3.0);
        targets.push_back(target);
    }
    if (targets.empty() && map.usedBounds.isValid()) {
        RebuildTarget fallback;
        fallback.region.bounds = map.usedBounds;
        fallback.region.center = map.usedBounds.center();
        fallback.region.area = map.usedBounds.area() * 0.15;
        fallback.importance = fallback.region.area;
        targets.push_back(fallback);
    }
    std::stable_sort(targets.begin(), targets.end(), [](const RebuildTarget& a, const RebuildTarget& b) {
        return a.importance > b.importance;
    });
    if (targets.size() > 10u) {
        targets.resize(10u);
    }
    return targets;
}

size_t targetSubsetSize(const EngineSettings& settings, size_t partCount, int attempt) {
    if (settings.performanceProfile == PerformanceProfile::Maximum) {
        const size_t normal = partCount >= 400 ? 36u : 24u;
        const size_t aggressive = partCount >= 400 ? 52u : 34u;
        return attempt % 5 == 4 ? aggressive : normal;
    }
    if (settings.performanceProfile == PerformanceProfile::Balanced) {
        return partCount >= 300 ? 24u : 16u;
    }
    return 8u;
}

double subsetPriority(
    size_t index,
    const Document& document,
    const LayoutState& state,
    const EmptySpaceMap& map,
    const RebuildTarget& target,
    size_t attempt) {
    const AABB box = transformedBounds(document.parts[index], state.poses[index]);
    const double area = partAreaScore(document.parts[index]);
    const double targetArea = std::max(1.0, target.region.area);
    double score = 0.0;
    if (box.width() <= target.region.bounds.width() && box.height() <= target.region.bounds.height()) {
        score += 170.0;
    }
    if (box.height() <= target.region.bounds.width() && box.width() <= target.region.bounds.height()) {
        score += 80.0;
    }
    score += std::min(1.0, area / targetArea) * 110.0;
    score += (1.0 / std::max(1.0, area)) * 5000.0;
    score -= distance(box.center(), target.region.center) * 0.015;
    if (map.usedBounds.isValid()) {
        const double eps = 8.0;
        if (std::abs(box.min.x - map.usedBounds.min.x) <= eps || std::abs(box.max.x - map.usedBounds.max.x) <= eps) {
            score += 130.0;
        }
        if (std::abs(box.min.y - map.usedBounds.min.y) <= eps || std::abs(box.max.y - map.usedBounds.max.y) <= eps) {
            score += 130.0;
        }
    }
    score += static_cast<double>((index * 2654435761u + attempt * 97u) & 0xffu) * 0.01;
    return score;
}

std::vector<size_t> selectSubset(
    const Document& document,
    const LayoutState& state,
    const EmptySpaceMap& map,
    const RebuildTarget& target,
    const EngineSettings& settings,
    size_t attempt) {
    const size_t count = std::min(document.parts.size(), state.poses.size());
    const size_t limit = std::min(count, targetSubsetSize(settings, count, static_cast<int>(attempt)));
    std::vector<size_t> order(count);
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return subsetPriority(a, document, state, map, target, attempt) >
            subsetPriority(b, document, state, map, target, attempt);
    });
    std::vector<size_t> subset;
    subset.reserve(limit);
    for (size_t index : order) {
        appendUnique(subset, index, limit);
    }
    return subset;
}

std::vector<Vec2> targetAnchors(const RebuildTarget& target, const EmptySpaceMap& map) {
    std::vector<Vec2> anchors;
    const AABB& b = target.region.bounds;
    const double ix = b.width() * 0.16;
    const double iy = b.height() * 0.16;
    anchors.push_back(target.region.center);
    anchors.push_back({b.min.x + ix, b.min.y + iy});
    anchors.push_back({b.max.x - ix, b.min.y + iy});
    anchors.push_back({b.min.x + ix, b.max.y - iy});
    anchors.push_back({b.max.x - ix, b.max.y - iy});
    anchors.push_back({b.center().x, b.min.y + iy});
    anchors.push_back({b.center().x, b.max.y - iy});
    anchors.push_back({b.min.x + ix, b.center().y});
    anchors.push_back({b.max.x - ix, b.center().y});
    for (size_t i = 0; i < map.regions.size() && i < 3u; ++i) {
        if (std::abs(map.regions[i].area - target.region.area) < 1e-6 &&
            distance(map.regions[i].center, target.region.center) < 1e-6) {
            continue;
        }
        anchors.push_back(map.regions[i].center);
    }
    return anchors;
}

std::vector<double> angleSamples(const EngineSettings& settings, const Pose& base) {
    std::vector<double> angles{base.angleRadians};
    if (!settings.allowRotation || settings.rotationMode == RotationMode::None) {
        return angles;
    }
    PoseSampler sampler;
    for (double angle : sampler.coarseRotationSamples(settings)) {
        if (std::none_of(angles.begin(), angles.end(), [&](double existing) { return std::abs(existing - angle) < 1e-9; })) {
            angles.push_back(angle);
        }
    }
    if (settings.performanceProfile == PerformanceProfile::Maximum) {
        const double fine = degreesToRadians(std::max(0.001, settings.rotationStepDegrees));
        angles.push_back(base.angleRadians + fine);
        angles.push_back(base.angleRadians - fine);
    }
    if (angles.size() > 12u) {
        angles.resize(12u);
    }
    return angles;
}

std::vector<bool> mirrorSamples(const EngineSettings& settings, const Pose& base) {
    std::vector<bool> mirrors{base.mirrored};
    if (settings.allowMirroring) {
        mirrors.push_back(!base.mirrored);
    }
    return mirrors;
}

std::vector<size_t> fixedPartsFor(const Document& document, const std::vector<size_t>& subset, const std::vector<size_t>& placedSubset) {
    std::vector<size_t> fixed;
    fixed.reserve(document.parts.size());
    for (size_t i = 0; i < document.parts.size(); ++i) {
        if (!containsIndex(subset, i) || containsIndex(placedSubset, i)) {
            fixed.push_back(i);
        }
    }
    return fixed;
}

double nodeRank(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const RebuildTarget& target,
    double contactPriority,
    const std::vector<size_t>& placed) {
    AABB used;
    double targetPull = 0.0;
    for (size_t i = 0; i < document.parts.size() && i < poses.size(); ++i) {
        const AABB box = transformedBounds(document.parts[i], poses[i]);
        used.include(box);
    }
    for (size_t index : placed) {
        if (index < document.parts.size() && index < poses.size()) {
            targetPull += distance(transformedBounds(document.parts[index], poses[index]).center(), target.region.center);
        }
    }
    const LayoutShapeMetrics metrics = computeLayoutShapeMetrics(document, settings, used);
    return std::max(1.0, used.area()) +
        metrics.towerScore * std::max(1.0, document.totalPartArea()) * 4.0 +
        targetPull * 0.35 -
        contactPriority * 700.0;
}

size_t maxAttemptsFor(const EngineSettings& settings, size_t partCount) {
    if (settings.performanceProfile == PerformanceProfile::Maximum) {
        return partCount >= 400 ? 45u : 60u;
    }
    if (settings.performanceProfile == PerformanceProfile::Balanced) {
        return partCount >= 300 ? 24u : 32u;
    }
    return 8u;
}

size_t placementDepthFor(const EngineSettings& settings, size_t subsetSize) {
    if (settings.performanceProfile == PerformanceProfile::Maximum) {
        return subsetSize >= 50u ? std::min<size_t>(subsetSize, 42u) : subsetSize;
    }
    if (settings.performanceProfile == PerformanceProfile::Balanced) {
        return std::min<size_t>(subsetSize, subsetSize >= 24u ? 20u : subsetSize);
    }
    return std::min<size_t>(subsetSize, 6u);
}

} // namespace

LayoutState ConstructiveRebuildEngine::optimize(
    const Document& document,
    const EngineSettings& settings,
    LayoutState initialValid,
    PenaltySystem& globalPenalties,
    const std::atomic_bool& stopRequested,
    SolverStats& stats,
    ConstructiveRebuildCallback callback) const {
    if (settings.performanceProfile == PerformanceProfile::Fast || document.parts.empty()) {
        return initialValid;
    }

    PenaltySystem attemptPenalties;
    Objective currentObjective = evaluateObjective(document, settings, initialValid.poses, attemptPenalties, globalPenalties);
    if (!currentObjective.state.valid()) {
        return initialValid;
    }
    LayoutState current = currentObjective.state;
    LayoutState best = current;
    double bestObjective = currentObjective.value;
    AdaptiveAcceptance acceptance(settings);
    AnalyticContactCandidateProvider candidateProvider;
    uint64_t version = 1;

    const auto started = Clock::now();
    const double safetyLimit = settings.timeLimitSeconds > 0.0
        ? std::max(4.0, settings.timeLimitSeconds * 0.58)
        : (settings.performanceProfile == PerformanceProfile::Maximum ? 45.0 : 22.0);
    const size_t attempts = maxAttemptsFor(settings, document.parts.size());

    for (size_t attempt = 0; attempt < attempts && !stopRequested.load(); ++attempt) {
        if (elapsedSeconds(started) >= safetyLimit) {
            break;
        }
        const EmptySpaceMap map = EmptySpaceAnalyzer{}.analyze(document, settings, current);
        std::vector<RebuildTarget> targets = makeTargets(map, document, settings);
        if (targets.empty()) {
            break;
        }
        const RebuildTarget& target = targets[attempt % targets.size()];
        std::vector<size_t> subset = selectSubset(document, current, map, target, settings, attempt);
        if (subset.size() < 4u) {
            continue;
        }
        ++stats.destroyAttempts;
        stats.destroySubsetTotal += subset.size();
        stats.averageSubsetSize = static_cast<double>(stats.destroySubsetTotal) / static_cast<double>(std::max<size_t>(1, stats.destroyAttempts));

        std::stable_sort(subset.begin(), subset.end(), [&](size_t a, size_t b) {
            const AABB boxA = transformedBounds(document.parts[a], current.poses[a]);
            const AABB boxB = transformedBounds(document.parts[b], current.poses[b]);
            const bool aFits = boxA.width() <= target.region.bounds.width() && boxA.height() <= target.region.bounds.height();
            const bool bFits = boxB.width() <= target.region.bounds.width() && boxB.height() <= target.region.bounds.height();
            if (aFits != bFits) {
                return aFits;
            }
            return partAreaScore(document.parts[a]) < partAreaScore(document.parts[b]);
        });

        const size_t depthLimit = placementDepthFor(settings, subset.size());
        stats.placementDepthTotal += depthLimit;
        stats.averagePlacementDepth = static_cast<double>(stats.placementDepthTotal) / static_cast<double>(std::max<size_t>(1, stats.destroyAttempts));
        const size_t beamWidth = settings.performanceProfile == PerformanceProfile::Maximum ? (document.parts.size() >= 400 ? 24u : 32u) : 16u;
        const size_t expansionLimit = settings.performanceProfile == PerformanceProfile::Maximum ? (document.parts.size() >= 400 ? 12u : 16u) : 10u;
        const size_t partialEvalLimit = settings.performanceProfile == PerformanceProfile::Maximum ? 4u : 2u;
        const size_t validLeafLimit = settings.performanceProfile == PerformanceProfile::Maximum ? (document.parts.size() >= 400 ? 100u : 160u) : 80u;
        const std::vector<Vec2> anchors = targetAnchors(target, map);

        BeamNode root;
        root.poses = current.poses;
        root.rank = nodeRank(document, settings, root.poses, target, 0.0, root.placed);
        std::vector<BeamNode> beam{std::move(root)};
        Objective bestAttempt;
        bool hasAttempt = false;

        for (size_t depth = 0; depth < depthLimit && !beam.empty() && !stopRequested.load(); ++depth) {
            const size_t partIndex = subset[depth];
            std::vector<BeamNode> expanded;
            expanded.reserve(beamWidth * expansionLimit);
            for (const BeamNode& node : beam) {
                ContactCandidateRequest request;
                request.movingPart = partIndex;
                request.fixedParts = fixedPartsFor(document, subset, node.placed);
                request.regionAnchors = anchors;
                request.angles = angleSamples(settings, current.poses[partIndex]);
                request.mirrors = mirrorSamples(settings, current.poses[partIndex]);
                request.ownerLimit = settings.performanceProfile == PerformanceProfile::Maximum ? 32u : 22u;
                request.perOwnerPointLimit = settings.performanceProfile == PerformanceProfile::Maximum ? 6u : 5u;
                request.candidateLimit = settings.performanceProfile == PerformanceProfile::Maximum ? 120u : 72u;
                ContactCandidateStats analyticStats;
                std::vector<ContactCandidate> candidates =
                    candidateProvider.generateCandidates(document, settings, node.poses, request, &analyticStats);
                if (partIndex < current.poses.size()) {
                    candidates.insert(candidates.begin(), {current.poses[partIndex], AnalyticContactKind::RegionAnchor, static_cast<size_t>(-1), -1, -1000.0});
                }
                stats.analyticCandidatesGenerated += analyticStats.generated;
                stats.analyticCandidatesValid += analyticStats.valid;
                stats.contactCandidatesRejectedCollision += analyticStats.rejectedCollision;
                stats.contactCandidatesRejectedClearance += analyticStats.rejectedClearance;
                stats.contactCandidatesRejectedSheet += analyticStats.rejectedSheet;

                size_t used = 0;
                for (const ContactCandidate& candidate : candidates) {
                    BeamNode next = node;
                    next.poses[partIndex] = candidate.pose;
                    next.placed.push_back(partIndex);
                    next.contactPriority = node.contactPriority + candidate.priority;
                    next.rank = nodeRank(document, settings, next.poses, target, next.contactPriority, next.placed);
                    expanded.push_back(std::move(next));
                    ++stats.beamNodesExpanded;
                    if (++used >= expansionLimit) {
                        break;
                    }
                }
            }
            std::stable_sort(expanded.begin(), expanded.end(), [](const BeamNode& a, const BeamNode& b) {
                if (std::abs(a.rank - b.rank) > 1e-9) {
                    return a.rank < b.rank;
                }
                return a.contactPriority > b.contactPriority;
            });
            if (expanded.size() > beamWidth) {
                expanded.resize(beamWidth);
            }
            const size_t evalCount = std::min(partialEvalLimit, expanded.size());
            for (size_t i = 0; i < evalCount; ++i) {
                const std::vector<size_t> changed = changedParts(expanded[i].poses, current.poses, subset);
                if (changed.empty()) {
                    continue;
                }
                Objective objective = evaluateObjective(document, settings, expanded[i].poses, attemptPenalties, globalPenalties);
                if (!objective.state.valid()) {
                    continue;
                }
                ++stats.beamValidLeaves;
                if (!hasAttempt || objective.value < bestAttempt.value) {
                    bestAttempt = std::move(objective);
                    hasAttempt = true;
                    if (callback) {
                        ActiveMoveSummary moves;
                        moves.region = changed.size();
                        ConstructiveRebuildProgress progress;
                        progress.current = bestAttempt.state;
                        progress.best = best;
                        progress.stats = stats;
                        progress.activeMoves = moves;
                        progress.versionId = ++version;
                        progress.layoutChanged = true;
                        progress.bestUpdated = false;
                        progress.changedParts = changed;
                        ++stats.rebuildPreviewEvents;
                        progress.stats = stats;
                        callback(progress);
                    }
                }
            }
            if (stats.beamValidLeaves >= validLeafLimit * std::max<size_t>(1, attempt + 1)) {
                break;
            }
            beam = std::move(expanded);
        }

        if (!hasAttempt) {
            continue;
        }

        const std::vector<size_t> changed = changedParts(bestAttempt.state.poses, current.poses, subset);
        if (changed.empty()) {
            continue;
        }

        const bool bestUpdate = betterQuality(bestAttempt.state, best, bestAttempt.value, bestObjective);
        bool accepted = bestUpdate;
        bool temporaryAccepted = false;
        if (!accepted) {
            AdaptiveAcceptanceContext context;
            context.currentScore = currentObjective.value;
            context.candidateScore = bestAttempt.value;
            context.bestScore = bestObjective;
            context.emptySpacePotential = bestAttempt.emptyMap.totalEmptyArea / sheetArea(document, settings);
            context.contactPotential = bestAttempt.state.contactReward / std::max(1.0, static_cast<double>(document.parts.size()));
            context.destroyRebuildMove = true;
            context.iteration = static_cast<int>(attempt);
            context.maxIterations = static_cast<int>(attempts);
            context.seed = settings.randomSeed == 0u ? 1u : settings.randomSeed;
            context.candidateIndex = attempt;
            const AdaptiveAcceptanceDecision decision = acceptance.decide(context);
            accepted = decision.accepted;
            temporaryAccepted = accepted;
            if (!accepted) {
                ++stats.rejectedByAcceptance;
            }
        }

        if (!accepted) {
            continue;
        }

        current = bestAttempt.state;
        currentObjective = std::move(bestAttempt);
        ++stats.destroyAccepted;
        ++stats.acceptedMoves;
        ++stats.activeMoveAcceptedTotal;
        if (bestUpdate) {
            best = current;
            bestObjective = currentObjective.value;
            ++stats.destroyBestUpdates;
            ++stats.bestUpdates;
            ++stats.acceptedBetter;
        } else if (temporaryAccepted) {
            ++stats.destroyTemporaryAccepted;
            ++stats.acceptedTemporary;
            ++stats.acceptedWorseMoves;
        }
        ActiveMoveSummary moves;
        moves.region = changed.size();
        ConstructiveRebuildProgress progress;
        progress.current = current;
        progress.best = best;
        progress.stats = stats;
        progress.activeMoves = moves;
        progress.versionId = ++version;
        progress.layoutChanged = true;
        progress.bestUpdated = bestUpdate;
        progress.changedParts = changed;
        if (callback) {
            callback(progress);
        }
    }

    const size_t acceptedTotal = stats.acceptedMoves;
    const size_t rejectedTotal = stats.rejectedByAcceptance + stats.rejectedWorseMoves + stats.rejectedByScore;
    if (acceptedTotal + rejectedTotal > 0) {
        stats.acceptanceRate = static_cast<double>(acceptedTotal) / static_cast<double>(acceptedTotal + rejectedTotal);
    }
    return best.valid() ? best : current;
}

} // namespace nest
