#include "engine/constructive_rebuild_engine.h"

#include "core/math_utils.h"
#include "engine/adaptive_acceptance.h"
#include "engine/empty_space_map.h"
#include "engine/contact_candidate_provider.h"
#include "engine/inner_fit_candidate_provider.h"
#include "engine/layout_score.h"
#include "engine/layout_score_components.h"
#include "engine/nfp_candidate_provider.h"
#include "engine/pose_sampler.h"
#include "geometry/clearance.h"
#include "geometry/collision.h"
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
    double usedArea = 0.0;
    double largestEmptyRegion = 0.0;
    double totalEmptyArea = 0.0;
    double contactCount = 0.0;
    double averageClearance = 0.0;
    double towerScore = 0.0;
};

struct LeafCandidate {
    Objective objective;
    double rank = std::numeric_limits<double>::max();
};

struct ClusterBeamState {
    std::vector<Pose> poses;
    std::vector<size_t> fixedParts;
    std::vector<size_t> placedClusterParts;
    std::vector<size_t> remainingClusterParts;
    double localRank = std::numeric_limits<double>::max();
    double contactGain = 0.0;
    double usedBoundsEstimate = 0.0;
    double emptyRegionCoverage = 0.0;
    double averageClearance = 0.0;
    size_t restoreFallbacks = 0;
};

struct ClusterBeamParams {
    size_t beamWidth = 4;
    size_t candidatePerPart = 8;
    size_t maxDepth = 4;
    size_t validLeafLimit = 16;
    size_t ownerLimit = 8;
    size_t pointLimit = 1;
    size_t candidateLimit = 24;
};

struct ClusterCandidateChoice {
    ContactCandidate candidate;
    bool restoreFallback = false;
};

bool isNfpCandidateKind(AnalyticContactKind kind) {
    return kind == AnalyticContactKind::NfpPartPart ||
        kind == AnalyticContactKind::NfpHoleBoundary ||
        kind == AnalyticContactKind::EdgeParallel;
}

bool isIfpCandidateKind(AnalyticContactKind kind) {
    return kind == AnalyticContactKind::InnerFitBoundary;
}

struct RebuildQualityDelta {
    double utilizationDelta = 0.0;
    double usedAreaDelta = 0.0;
    double usedWidthDelta = 0.0;
    double usedHeightDelta = 0.0;
    double largestEmptyRegionDelta = 0.0;
    double totalEmptyAreaDelta = 0.0;
    double contactCountDelta = 0.0;
    double averageClearanceDelta = 0.0;
    double towerScoreDelta = 0.0;
    double objectiveDelta = 0.0;
};

enum class RebuildDecisionReason {
    BetterUtilization,
    ReducedUsedBounds,
    ReducedLargestEmptyRegion,
    ReducedTotalEmptyArea,
    IncreasedContactWithGapReduction,
    TemporaryObjectiveGain,
    RejectedNoObjectiveGain,
    RejectedInvalid,
    RejectedWorseAllMetrics
};

struct DepthBudget {
    size_t nodesToExpand = 1;
    size_t expansionLimit = 1;
    size_t partialEvalLimit = 1;
    size_t ownerLimit = 4;
    size_t pointLimit = 1;
    size_t candidateLimit = 8;
    bool analytic = false;
    bool removeUnplacedSubset = false;
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

std::vector<size_t> changedPartsAll(const std::vector<Pose>& a, const std::vector<Pose>& b) {
    std::vector<size_t> changed;
    const size_t count = std::min(a.size(), b.size());
    for (size_t index = 0; index < count; ++index) {
        if (poseChanged(a[index], b[index])) {
            changed.push_back(index);
        }
    }
    return changed;
}

double safeUsedArea(const LayoutState& state) {
    return std::max(1.0, state.usedWidth * state.usedHeight);
}

RebuildQualityDelta qualityDelta(const Objective& before, const Objective& after) {
    RebuildQualityDelta delta;
    delta.utilizationDelta = after.state.utilization - before.state.utilization;
    delta.usedAreaDelta = after.usedArea - before.usedArea;
    delta.usedWidthDelta = after.state.usedWidth - before.state.usedWidth;
    delta.usedHeightDelta = after.state.usedHeight - before.state.usedHeight;
    delta.largestEmptyRegionDelta = after.largestEmptyRegion - before.largestEmptyRegion;
    delta.totalEmptyAreaDelta = after.totalEmptyArea - before.totalEmptyArea;
    delta.contactCountDelta = after.contactCount - before.contactCount;
    delta.averageClearanceDelta = after.averageClearance - before.averageClearance;
    delta.towerScoreDelta = after.towerScore - before.towerScore;
    delta.objectiveDelta = after.value - before.value;
    return delta;
}

bool reducedUsedBounds(const RebuildQualityDelta& delta) {
    return delta.usedAreaDelta < -1.0 ||
        delta.usedWidthDelta < -0.25 ||
        delta.usedHeightDelta < -0.25;
}

bool reducedLargestEmptyRegion(const RebuildQualityDelta& delta) {
    return delta.largestEmptyRegionDelta < -1.0;
}

bool reducedTotalEmptyArea(const RebuildQualityDelta& delta) {
    return delta.totalEmptyAreaDelta < -1.0;
}

bool increasedContactWithGapReduction(const RebuildQualityDelta& delta) {
    const bool contactGain = delta.contactCountDelta > 0.05;
    return contactGain && (reducedLargestEmptyRegion(delta) || reducedTotalEmptyArea(delta) || reducedUsedBounds(delta));
}

bool hasObjectiveGain(const RebuildQualityDelta& delta) {
    if (reducedUsedBounds(delta) ||
        reducedLargestEmptyRegion(delta) ||
        reducedTotalEmptyArea(delta) ||
        increasedContactWithGapReduction(delta)) {
        return true;
    }
    const bool noMajorEmptyRegression =
        delta.usedAreaDelta <= 100.0 &&
        delta.largestEmptyRegionDelta <= 100.0 &&
        delta.totalEmptyAreaDelta <= 100.0;
    if ((delta.averageClearanceDelta < -0.01 || delta.towerScoreDelta < -0.001) && noMajorEmptyRegression) {
        return true;
    }
    if (delta.utilizationDelta > 0.00001 &&
        delta.usedAreaDelta <= 10.0 &&
        delta.largestEmptyRegionDelta <= 10.0 &&
        delta.totalEmptyAreaDelta <= 10.0) {
        return true;
    }
    return false;
}

double leafObjectiveRank(const Objective& before, const Objective& after) {
    const RebuildQualityDelta delta = qualityDelta(before, after);
    const bool gapOrBoundsGain = reducedUsedBounds(delta) || reducedLargestEmptyRegion(delta) || reducedTotalEmptyArea(delta);
    const double contactCredit = gapOrBoundsGain ? delta.contactCountDelta * 1800.0 : delta.contactCountDelta * 120.0;
    return
        delta.usedAreaDelta * 3.5 +
        delta.usedWidthDelta * 220.0 +
        delta.usedHeightDelta * 220.0 +
        delta.largestEmptyRegionDelta * 22.0 +
        delta.totalEmptyAreaDelta * 3.5 +
        delta.towerScoreDelta * 65000.0 +
        delta.averageClearanceDelta * 800.0 -
        contactCredit -
        delta.utilizationDelta * 180000.0 +
        std::max(0.0, delta.objectiveDelta) * 0.05;
}

void rememberLeafCandidate(std::vector<LeafCandidate>& leaves, const Objective& objective, double rank, size_t limit) {
    leaves.push_back({objective, rank});
    std::stable_sort(leaves.begin(), leaves.end(), [](const LeafCandidate& a, const LeafCandidate& b) {
        return a.rank < b.rank;
    });
    if (leaves.size() > limit) {
        leaves.resize(limit);
    }
}

bool betterQuality(const Objective& candidate, const Objective& incumbent, RebuildQualityDelta* outDelta = nullptr) {
    RebuildQualityDelta delta = qualityDelta(incumbent, candidate);
    if (outDelta != nullptr) {
        *outDelta = delta;
    }
    if (!candidate.state.valid()) {
        return false;
    }
    if (!incumbent.state.valid()) {
        return true;
    }

    const bool usedGain = reducedUsedBounds(delta);
    const bool utilizationGain = delta.utilizationDelta > 0.0001;

    if (!usedGain && !utilizationGain) {
        return false;
    }
    if (candidate.state.utilization + 0.003 < incumbent.state.utilization) {
        return false;
    }
    if (usedGain || utilizationGain) {
        return true;
    }
    return false;
}

RebuildDecisionReason classifyGain(const RebuildQualityDelta& delta, bool temporary) {
    if (temporary) {
        return RebuildDecisionReason::TemporaryObjectiveGain;
    }
    if (delta.utilizationDelta > 0.00001) {
        return RebuildDecisionReason::BetterUtilization;
    }
    if (reducedUsedBounds(delta)) {
        return RebuildDecisionReason::ReducedUsedBounds;
    }
    if (reducedLargestEmptyRegion(delta)) {
        return RebuildDecisionReason::ReducedLargestEmptyRegion;
    }
    if (reducedTotalEmptyArea(delta)) {
        return RebuildDecisionReason::ReducedTotalEmptyArea;
    }
    if (increasedContactWithGapReduction(delta)) {
        return RebuildDecisionReason::IncreasedContactWithGapReduction;
    }
    return RebuildDecisionReason::RejectedWorseAllMetrics;
}

void recordAttemptMetrics(SolverStats& stats, const Objective& before, const Objective& after) {
    stats.rebuildBeforeUtilization = before.state.utilization;
    stats.rebuildAfterUtilization = after.state.utilization;
    stats.rebuildBeforeUsedArea = before.usedArea;
    stats.rebuildAfterUsedArea = after.usedArea;
    stats.rebuildBeforeLargestEmptyRegion = before.largestEmptyRegion;
    stats.rebuildAfterLargestEmptyRegion = after.largestEmptyRegion;
    stats.rebuildBeforeTotalEmptyArea = before.totalEmptyArea;
    stats.rebuildAfterTotalEmptyArea = after.totalEmptyArea;
    stats.rebuildBeforeContactCount = before.contactCount;
    stats.rebuildAfterContactCount = after.contactCount;
    stats.rebuildBeforeAverageClearance = before.averageClearance;
    stats.rebuildAfterAverageClearance = after.averageClearance;
    stats.rebuildBeforeTowerScore = before.towerScore;
    stats.rebuildAfterTowerScore = after.towerScore;

    const RebuildQualityDelta delta = qualityDelta(before, after);
    stats.bestRebuildUsedAreaReduction = std::max(stats.bestRebuildUsedAreaReduction, std::max(0.0, -delta.usedAreaDelta));
    stats.bestRebuildUsedWidthReduction = std::max(stats.bestRebuildUsedWidthReduction, std::max(0.0, -delta.usedWidthDelta));
    stats.bestRebuildUsedHeightReduction = std::max(stats.bestRebuildUsedHeightReduction, std::max(0.0, -delta.usedHeightDelta));
    stats.bestRebuildUtilizationGain = std::max(stats.bestRebuildUtilizationGain, std::max(0.0, delta.utilizationDelta));
    stats.bestRebuildLargestEmptyRegionReduction = std::max(stats.bestRebuildLargestEmptyRegionReduction, std::max(0.0, -delta.largestEmptyRegionDelta));
    stats.bestRebuildTotalEmptyAreaReduction = std::max(stats.bestRebuildTotalEmptyAreaReduction, std::max(0.0, -delta.totalEmptyAreaDelta));
    stats.bestRebuildContactGain = std::max(stats.bestRebuildContactGain, std::max(0.0, delta.contactCountDelta));
}

void recordAcceptedReason(SolverStats& stats, RebuildDecisionReason reason) {
    switch (reason) {
    case RebuildDecisionReason::ReducedUsedBounds:
        ++stats.destroyAcceptedReducedUsedBounds;
        break;
    case RebuildDecisionReason::ReducedLargestEmptyRegion:
        ++stats.destroyAcceptedReducedLargestEmptyRegion;
        break;
    case RebuildDecisionReason::ReducedTotalEmptyArea:
        ++stats.destroyAcceptedReducedTotalEmptyArea;
        break;
    case RebuildDecisionReason::IncreasedContactWithGapReduction:
        ++stats.destroyAcceptedIncreasedContactWithGapReduction;
        break;
    default:
        break;
    }
}

double intersectionArea(const AABB& a, const AABB& b) {
    if (!a.isValid() || !b.isValid()) {
        return 0.0;
    }
    const double minX = std::max(a.min.x, b.min.x);
    const double maxX = std::min(a.max.x, b.max.x);
    const double minY = std::max(a.min.y, b.min.y);
    const double maxY = std::min(a.max.y, b.max.y);
    if (maxX <= minX || maxY <= minY) {
        return 0.0;
    }
    return (maxX - minX) * (maxY - minY);
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
    const double usedArea = safeUsedArea(objective.state);
    const double totalArea = std::max(1.0, document.totalPartArea());
    const LayoutShapeMetrics shapeMetrics = computeLayoutShapeMetrics(document, settings, used);
    objective.usedArea = usedArea;
    objective.largestEmptyRegion = objective.emptyMap.largestRegionArea;
    objective.totalEmptyArea = objective.emptyMap.totalEmptyArea;
    objective.contactCount = objective.state.contactReward;
    objective.averageClearance = 0.0;
    objective.towerScore = shapeMetrics.towerScore;
    const double largestGapWeight = settings.performanceProfile == PerformanceProfile::Maximum ? 16.0 : 7.5;
    const double emptyWeight = settings.performanceProfile == PerformanceProfile::Maximum ? 3.2 : 1.6;
    const double boundsWeight = settings.performanceProfile == PerformanceProfile::Maximum ? 3.6 : 1.9;
    const double contactWeight = settings.performanceProfile == PerformanceProfile::Maximum ? 1700.0 : 950.0;
    objective.value = objective.state.totalScore +
        objective.emptyMap.largestRegionArea * largestGapWeight +
        objective.emptyMap.totalEmptyArea * emptyWeight +
        usedArea * boundsWeight +
        shapeMetrics.towerScore * totalArea * 12.0 +
        static_cast<double>(objective.emptyMap.fillableRegionCount(std::max(1.0, totalArea / std::max<size_t>(1, document.parts.size()) * 0.35))) * totalArea * 0.02 -
        objective.state.contactReward * contactWeight -
        objective.state.utilization * 220000.0;
    return objective;
}

std::vector<RebuildTarget> makeTargets(const EmptySpaceMap& map, const Document& document, const EngineSettings& settings) {
    std::vector<RebuildTarget> targets;
    const double usableArea = sheetArea(document, settings);
    for (const EmptyRegion& region : map.regions) {
        RebuildTarget target;
        target.region = region;
        const double fillability = std::min(1.0, region.area / std::max(1.0, document.totalPartArea() * 0.06));
        const double boundaryBias = region.touchesUsedBoundary ? 0.75 : 0.0;
        const double narrowness = region.bounds.height() > 1e-9 && region.bounds.width() > 1e-9
            ? std::min(region.bounds.width(), region.bounds.height()) / std::max(region.bounds.width(), region.bounds.height())
            : 0.0;
        const double sizeBias = region.area / usableArea;
        target.importance = region.area * (1.0 + fillability * 2.0 + boundaryBias + sizeBias * 4.0 + narrowness * 0.4);
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
        score += 260.0;
    }
    if (box.height() <= target.region.bounds.width() && box.width() <= target.region.bounds.height()) {
        score += 140.0;
    }
    const double areaFit = std::min(area, targetArea) / targetArea;
    score += areaFit * 190.0;
    score += (1.0 / std::max(1.0, area)) * 3500.0;
    score -= distance(box.center(), target.region.center) * 0.010;
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

std::vector<size_t> fixedPartsFor(
    const Document& document,
    const std::vector<size_t>& subset,
    const std::vector<size_t>& placedSubset,
    size_t movingPart,
    bool removeUnplacedSubset) {
    std::vector<size_t> fixed;
    fixed.reserve(document.parts.size());
    for (size_t i = 0; i < document.parts.size(); ++i) {
        if (i == movingPart) {
            continue;
        }
        if (removeUnplacedSubset && containsIndex(subset, i) && !containsIndex(placedSubset, i)) {
            continue;
        }
        fixed.push_back(i);
    }
    return fixed;
}

double nodeRank(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const RebuildTarget& target,
    double contactPriority,
    const std::vector<size_t>& placed,
    const std::vector<size_t>& activeParts) {
    AABB used;
    double targetPull = 0.0;
    double targetCoverage = 0.0;
    double movedArea = 0.0;
    for (size_t index : activeParts) {
        if (index >= document.parts.size() || index >= poses.size()) {
            continue;
        }
        const AABB box = transformedBounds(document.parts[index], poses[index]);
        used.include(box);
    }
    for (size_t index : placed) {
        if (index < document.parts.size() && index < poses.size()) {
            const AABB box = transformedBounds(document.parts[index], poses[index]);
            targetPull += distance(box.center(), target.region.center);
            targetCoverage += intersectionArea(box, target.region.bounds);
            movedArea += std::max(1.0, partAreaScore(document.parts[index]));
        }
    }
    const LayoutShapeMetrics metrics = computeLayoutShapeMetrics(document, settings, used);
    const double coverageRatio = movedArea > 1e-9 ? targetCoverage / movedArea : 0.0;
    return std::max(1.0, used.area()) +
        metrics.towerScore * std::max(1.0, document.totalPartArea()) * 4.0 +
        targetPull * 0.35 -
        targetCoverage * 22.0 -
        coverageRatio * std::max(1.0, target.region.area) * 5.0 -
        contactPriority * 900.0;
}

double nodeRank(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const RebuildTarget& target,
    double contactPriority,
    const std::vector<size_t>& placed) {
    std::vector<size_t> activeParts;
    activeParts.reserve(std::min(document.parts.size(), poses.size()));
    for (size_t i = 0; i < document.parts.size() && i < poses.size(); ++i) {
        activeParts.push_back(i);
    }
    return nodeRank(document, settings, poses, target, contactPriority, placed, activeParts);
}

double candidateSelectionScore(
    const Document& document,
    size_t partIndex,
    const Pose& pose,
    const AABB& currentUsed,
    const RebuildTarget& target,
    double basePriority) {
    if (partIndex >= document.parts.size()) {
        return basePriority;
    }
    const AABB box = transformedBounds(document.parts[partIndex], pose);
    const double partArea = std::max(1.0, partAreaScore(document.parts[partIndex]));
    const double targetOverlapRatio = intersectionArea(box, target.region.bounds) / partArea;
    AABB expandedUsed = currentUsed;
    expandedUsed.include(box);
    const double expansionPenalty = currentUsed.isValid()
        ? std::max(0.0, expandedUsed.area() - currentUsed.area())
        : 0.0;
    const double distancePenalty = distance(box.center(), target.region.center);
    const bool fitsTargetBounds =
        (box.width() <= target.region.bounds.width() && box.height() <= target.region.bounds.height()) ||
        (box.height() <= target.region.bounds.width() && box.width() <= target.region.bounds.height());
    return basePriority +
        targetOverlapRatio * 420.0 +
        (fitsTargetBounds ? 80.0 : 0.0) -
        expansionPenalty * 0.020 -
        distancePenalty * 0.12;
}

bool acceptableConstructiveCandidate(
    const Document& document,
    size_t partIndex,
    const Pose& pose,
    const AABB& currentUsed,
    const RebuildTarget& target) {
    if (partIndex >= document.parts.size() || !currentUsed.isValid()) {
        return true;
    }
    const AABB box = transformedBounds(document.parts[partIndex], pose);
    const double partArea = std::max(1.0, partAreaScore(document.parts[partIndex]));
    const double targetOverlapRatio = intersectionArea(box, target.region.bounds) / partArea;
    AABB expandedUsed = currentUsed;
    expandedUsed.include(box);
    const double growthRatio = expandedUsed.area() / std::max(1.0, currentUsed.area());
    const AABB softUsed = currentUsed.expanded(std::max(4.0, std::sqrt(partArea) * 0.35));
    const bool nearCurrentCluster =
        box.min.x >= softUsed.min.x && box.max.x <= softUsed.max.x &&
        box.min.y >= softUsed.min.y && box.max.y <= softUsed.max.y;
    return nearCurrentCluster || targetOverlapRatio >= 0.08 || growthRatio <= 1.12;
}

bool poseValidInLayout(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    size_t partIndex,
    const Pose& pose) {
    if (partIndex >= document.parts.size()) {
        return false;
    }
    const Part& part = document.parts[partIndex];
    if (!isPartInsideSheet(part, pose, document.sheet, settings.collisionTolerance) ||
        !partRespectsSheetClearance(part, pose, document.sheet, settings.margin, settings.collisionTolerance)) {
        return false;
    }
    const AABB bounds = transformedBounds(part, pose).expanded(settings.partSpacing + settings.collisionTolerance);
    for (size_t i = 0; i < document.parts.size() && i < poses.size(); ++i) {
        if (i == partIndex) {
            continue;
        }
        const AABB otherBounds = transformedBounds(document.parts[i], poses[i]);
        if (!bounds.overlaps(otherBounds, settings.collisionTolerance)) {
            continue;
        }
        if (partsCollide(part, pose, document.parts[i], poses[i], settings.collisionTolerance) ||
            !partsRespectClearance(part, pose, document.parts[i], poses[i], settings.partSpacing, settings.collisionTolerance)) {
            return false;
        }
    }
    return true;
}

bool poseValidAgainstParts(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    size_t partIndex,
    const Pose& pose,
    const std::vector<size_t>& checkParts) {
    if (partIndex >= document.parts.size()) {
        return false;
    }
    const Part& part = document.parts[partIndex];
    if (!isPartInsideSheet(part, pose, document.sheet, settings.collisionTolerance) ||
        !partRespectsSheetClearance(part, pose, document.sheet, settings.margin, settings.collisionTolerance)) {
        return false;
    }
    const AABB bounds = transformedBounds(part, pose).expanded(settings.partSpacing + settings.collisionTolerance);
    for (size_t other : checkParts) {
        if (other == partIndex || other >= document.parts.size() || other >= poses.size()) {
            continue;
        }
        const AABB otherBounds = transformedBounds(document.parts[other], poses[other]);
        if (!bounds.overlaps(otherBounds, settings.collisionTolerance)) {
            continue;
        }
        if (partsCollide(part, pose, document.parts[other], poses[other], settings.collisionTolerance) ||
            !partsRespectClearance(part, pose, document.parts[other], poses[other], settings.partSpacing, settings.collisionTolerance)) {
            return false;
        }
    }
    return true;
}

Pose translatedPose(Pose pose, Vec2 direction, double distance) {
    pose.x += direction.x * distance;
    pose.y += direction.y * distance;
    return pose;
}

bool slidePoseSafelyAgainstParts(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    size_t partIndex,
    Vec2 direction,
    double maxDistance,
    const std::vector<size_t>& checkParts,
    Pose& outPose) {
    if (partIndex >= poses.size()) {
        return false;
    }
    const double len = direction.length();
    if (len <= 1e-9 || maxDistance <= 1e-9) {
        return false;
    }
    direction = direction / len;
    const Pose start = poses[partIndex];
    if (!poseValidAgainstParts(document, settings, poses, partIndex, start, checkParts)) {
        return false;
    }

    double low = 0.0;
    double high = std::max(0.0, maxDistance);
    double probe = std::min(high, std::max(0.5, high / 16.0));
    while (probe < high && poseValidAgainstParts(document, settings, poses, partIndex, translatedPose(start, direction, probe), checkParts)) {
        low = probe;
        probe = std::min(high, probe * 2.0);
    }
    if (poseValidAgainstParts(document, settings, poses, partIndex, translatedPose(start, direction, probe), checkParts)) {
        low = probe;
    } else {
        high = probe;
    }
    for (int i = 0; i < 22; ++i) {
        const double mid = (low + high) * 0.5;
        if (poseValidAgainstParts(document, settings, poses, partIndex, translatedPose(start, direction, mid), checkParts)) {
            low = mid;
        } else {
            high = mid;
        }
    }
    if (low <= std::max(0.01, settings.collisionTolerance)) {
        return false;
    }
    outPose = translatedPose(start, direction, low);
    return true;
}

bool slidePoseSafely(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    size_t partIndex,
    Vec2 direction,
    double maxDistance,
    Pose& outPose) {
    if (partIndex >= poses.size()) {
        return false;
    }
    const double len = direction.length();
    if (len <= 1e-9 || maxDistance <= 1e-9) {
        return false;
    }
    direction = direction / len;
    const Pose start = poses[partIndex];
    if (!poseValidInLayout(document, settings, poses, partIndex, start)) {
        return false;
    }

    double low = 0.0;
    double high = std::max(0.0, maxDistance);
    double probe = std::min(high, std::max(0.5, high / 16.0));
    while (probe < high && poseValidInLayout(document, settings, poses, partIndex, translatedPose(start, direction, probe))) {
        low = probe;
        probe = std::min(high, probe * 2.0);
    }
    if (poseValidInLayout(document, settings, poses, partIndex, translatedPose(start, direction, probe))) {
        low = probe;
    } else {
        high = probe;
    }
    for (int i = 0; i < 22; ++i) {
        const double mid = (low + high) * 0.5;
        if (poseValidInLayout(document, settings, poses, partIndex, translatedPose(start, direction, mid))) {
            low = mid;
        } else {
            high = mid;
        }
    }
    if (low <= std::max(0.01, settings.collisionTolerance)) {
        return false;
    }
    outPose = translatedPose(start, direction, low);
    return true;
}

std::vector<Vec2> compactionDirections(
    const Document& document,
    const EngineSettings& settings,
    const LayoutState& state,
    size_t partIndex,
    const RebuildTarget& target) {
    const double hx = [&]() {
        switch (settings.placementStrategy) {
        case PlacementStrategy::BottomRight:
        case PlacementStrategy::TopRight:
        case PlacementStrategy::RightToLeft:
            return 1.0;
        default:
            return -1.0;
        }
    }();
    const double vy = [&]() {
        switch (settings.placementStrategy) {
        case PlacementStrategy::TopLeft:
        case PlacementStrategy::TopRight:
        case PlacementStrategy::TopToBottom:
            return 1.0;
        default:
            return -1.0;
        }
    }();
    std::vector<Vec2> directions{{hx, 0.0}, {0.0, vy}, {hx, vy}};
    const Vec2 cardinals[] = {
        {1.0, 0.0}, {-1.0, 0.0}, {0.0, 1.0}, {0.0, -1.0},
        {1.0, 1.0}, {-1.0, 1.0}, {1.0, -1.0}, {-1.0, -1.0}
    };
    directions.insert(directions.end(), std::begin(cardinals), std::end(cardinals));
    if (partIndex < document.parts.size() && partIndex < state.poses.size()) {
        const AABB box = transformedBounds(document.parts[partIndex], state.poses[partIndex]);
        const Vec2 center = box.center();
        const Vec2 sheetCenter{
            document.sheet.origin.x + document.sheet.width * 0.5,
            document.sheet.origin.y + document.sheet.height * 0.5
        };
        if (settings.placementStrategy == PlacementStrategy::CenterOut ||
            settings.placementStrategy == PlacementStrategy::OutsideIn) {
            directions.insert(directions.begin(), sheetCenter - center);
        }
        directions.insert(directions.begin(), target.region.center - center);
    }
    std::vector<Vec2> unique;
    for (Vec2 direction : directions) {
        const double len = direction.length();
        if (len <= 1e-9) {
            continue;
        }
        direction = direction / len;
        const bool exists = std::any_of(unique.begin(), unique.end(), [&](Vec2 existing) {
            return distance(existing, direction) < 1e-6;
        });
        if (!exists) {
            unique.push_back(direction);
        }
    }
    return unique;
}

std::vector<size_t> compactionCluster(
    const Document& document,
    const EngineSettings& settings,
    const LayoutState& state,
    const std::vector<size_t>& changed,
    const std::vector<size_t>& subset,
    const RebuildTarget& target,
    const EmptySpaceMap& map) {
    std::vector<size_t> seeds = changed.empty() ? subset : changed;
    AABB seedBounds;
    for (size_t index : seeds) {
        if (index < document.parts.size() && index < state.poses.size()) {
            seedBounds.include(transformedBounds(document.parts[index], state.poses[index]));
        }
    }
    if (!seedBounds.isValid()) {
        seedBounds = target.region.bounds;
    }
    const double expand = std::max(24.0, settings.partSpacing * 6.0);
    const AABB neighborhood = seedBounds.expanded(expand);
    const AABB targetNeighborhood = target.region.bounds.expanded(expand);
    const size_t limit = document.parts.size() >= 400 ? 24u : 20u;

    std::vector<std::pair<double, size_t>> scored;
    scored.reserve(document.parts.size());
    for (size_t i = 0; i < document.parts.size() && i < state.poses.size(); ++i) {
        const AABB box = transformedBounds(document.parts[i], state.poses[i]);
        const bool isChanged = containsIndex(changed, i);
        const bool inSubset = containsIndex(subset, i);
        const bool nearChanged = box.overlaps(neighborhood, 0.0);
        const bool nearTarget = box.overlaps(targetNeighborhood, 0.0);
        bool boundaryContributor = false;
        if (map.usedBounds.isValid()) {
            const double eps = std::max(3.0, settings.partSpacing + 1.0);
            boundaryContributor =
                std::abs(box.min.x - map.usedBounds.min.x) <= eps ||
                std::abs(box.max.x - map.usedBounds.max.x) <= eps ||
                std::abs(box.min.y - map.usedBounds.min.y) <= eps ||
                std::abs(box.max.y - map.usedBounds.max.y) <= eps;
        }
        if (!isChanged && !inSubset && !nearChanged && !nearTarget && !boundaryContributor) {
            continue;
        }
        double score = distance(box.center(), target.region.center);
        if (isChanged) {
            score -= 5000.0;
        }
        if (inSubset) {
            score -= 2500.0;
        }
        if (boundaryContributor) {
            score -= 1200.0;
        }
        if (nearTarget) {
            score -= 700.0;
        }
        scored.push_back({score, i});
    }
    std::stable_sort(scored.begin(), scored.end(), [](const auto& a, const auto& b) {
        return a.first < b.first;
    });
    std::vector<size_t> cluster;
    cluster.reserve(std::min(limit, scored.size()));
    for (const auto& item : scored) {
        appendUnique(cluster, item.second, limit);
    }
    return cluster;
}

AABB boundsForParts(const Document& document, const std::vector<Pose>& poses, const std::vector<size_t>& parts) {
    AABB bounds;
    for (size_t index : parts) {
        if (index < document.parts.size() && index < poses.size()) {
            bounds.include(transformedBounds(document.parts[index], poses[index]));
        }
    }
    return bounds;
}

std::vector<size_t> outsideClusterParts(const Document& document, const std::vector<Pose>& poses, const std::vector<size_t>& cluster) {
    std::vector<size_t> fixed;
    fixed.reserve(std::min(document.parts.size(), poses.size()));
    for (size_t i = 0; i < document.parts.size() && i < poses.size(); ++i) {
        if (!containsIndex(cluster, i)) {
            fixed.push_back(i);
        }
    }
    return fixed;
}

std::vector<size_t> clusterReinsertOrder(
    const Document& document,
    const LayoutState& state,
    const std::vector<size_t>& cluster,
    const EmptySpaceMap& map,
    const RebuildTarget& target) {
    std::vector<size_t> order = cluster;
    std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        const AABB boxA = transformedBounds(document.parts[a], state.poses[a]);
        const AABB boxB = transformedBounds(document.parts[b], state.poses[b]);
        auto priority = [&](size_t index, const AABB& box) {
            double value = 0.0;
            if (map.usedBounds.isValid()) {
                const double eps = 8.0;
                if (std::abs(box.min.x - map.usedBounds.min.x) <= eps || std::abs(box.max.x - map.usedBounds.max.x) <= eps ||
                    std::abs(box.min.y - map.usedBounds.min.y) <= eps || std::abs(box.max.y - map.usedBounds.max.y) <= eps) {
                    value += 2000.0;
                }
            }
            value += partAreaScore(document.parts[index]) * 0.08;
            value -= distance(box.center(), target.region.center) * 0.02;
            if (box.width() <= target.region.bounds.width() && box.height() <= target.region.bounds.height()) {
                value += 500.0;
            }
            return value;
        };
        return priority(a, boxA) > priority(b, boxB);
    });
    return order;
}

double localPlacementScore(
    const Document& document,
    size_t partIndex,
    const Pose& pose,
    const AABB& fixedBounds,
    const RebuildTarget& target,
    double candidatePriority) {
    if (partIndex >= document.parts.size()) {
        return std::numeric_limits<double>::max();
    }
    const AABB box = transformedBounds(document.parts[partIndex], pose);
    AABB merged = fixedBounds;
    merged.include(box);
    const double expansion = fixedBounds.isValid() ? std::max(0.0, merged.area() - fixedBounds.area()) : merged.area();
    const double overlap = intersectionArea(box, target.region.bounds);
    const double area = std::max(1.0, partAreaScore(document.parts[partIndex]));
    return expansion * 0.020 +
        distance(box.center(), target.region.center) * 0.10 -
        (overlap / area) * 380.0 -
        candidatePriority * 1.5;
}

Pose contactLockedPose(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    size_t partIndex,
    Pose pose,
    const std::vector<size_t>& checkParts,
    const RebuildTarget& target,
    const AABB& fixedBounds) {
    if (!poseValidAgainstParts(document, settings, poses, partIndex, pose, checkParts)) {
        return pose;
    }
    Pose best = pose;
    double bestScore = localPlacementScore(document, partIndex, best, fixedBounds, target, 0.0);
    std::vector<Vec2> directions{{1.0, 0.0}, {-1.0, 0.0}, {0.0, 1.0}, {0.0, -1.0},
        {1.0, 1.0}, {-1.0, 1.0}, {1.0, -1.0}, {-1.0, -1.0}};
    if (partIndex < document.parts.size()) {
        const AABB box = transformedBounds(document.parts[partIndex], pose);
        directions.insert(directions.begin(), target.region.center - box.center());
        if (fixedBounds.isValid()) {
            directions.insert(directions.begin(), fixedBounds.center() - box.center());
        }
    }
    std::vector<Pose> trialPoses = poses;
    trialPoses[partIndex] = pose;
    const double maxDistance = std::max(8.0, std::min(document.sheet.width, document.sheet.height) * 0.08);
    for (Vec2 direction : directions) {
        Pose locked;
        if (!slidePoseSafelyAgainstParts(document, settings, trialPoses, partIndex, direction, maxDistance, checkParts, locked)) {
            continue;
        }
        const double score = localPlacementScore(document, partIndex, locked, fixedBounds, target, 0.0);
        if (score + 1e-9 < bestScore) {
            bestScore = score;
            best = locked;
        }
    }
    return best;
}

ClusterBeamParams clusterBeamParamsFor(const EngineSettings& settings, size_t partCount, size_t clusterSize, bool denseSmallMode) {
    ClusterBeamParams params;
    if (settings.performanceProfile == PerformanceProfile::Maximum) {
        params.beamWidth = partCount >= 400u ? 16u : 18u;
        params.candidatePerPart = denseSmallMode ? 16u : 12u;
        params.maxDepth = clusterSize;
        params.validLeafLimit = partCount >= 400u ? 50u : 70u;
        params.ownerLimit = partCount >= 400u ? 10u : 12u;
        params.pointLimit = 2u;
        params.candidateLimit = denseSmallMode ? 40u : 32u;
        return params;
    }
    if (settings.performanceProfile == PerformanceProfile::Balanced) {
        params.beamWidth = partCount >= 300u ? 8u : 12u;
        params.candidatePerPart = denseSmallMode ? 12u : 10u;
        params.maxDepth = clusterSize;
        params.validLeafLimit = 40u;
        params.ownerLimit = 10u;
        params.pointLimit = 2u;
        params.candidateLimit = denseSmallMode ? 32u : 28u;
        return params;
    }
    params.beamWidth = 4u;
    params.candidatePerPart = denseSmallMode ? 12u : 8u;
    params.maxDepth = std::min<size_t>(clusterSize, 8u);
    params.validLeafLimit = 24u;
    params.ownerLimit = 8u;
    params.pointLimit = 1u;
    params.candidateLimit = 24u;
    return params;
}

void removePartIndex(std::vector<size_t>& values, size_t partIndex) {
    values.erase(std::remove(values.begin(), values.end(), partIndex), values.end());
}

AABB partialBoundsFor(const Document& document, const std::vector<Pose>& poses, const std::vector<size_t>& activeParts);
std::vector<size_t> activePartsForClusterState(const ClusterBeamState& state, size_t limit);

double partReinsertNeedScore(
    const Document& document,
    const std::vector<Pose>& poses,
    size_t index,
    const EmptySpaceMap& map,
    const RebuildTarget& target,
    bool denseSmallMode,
    size_t placedCount,
    const AABB& activeBounds) {
    if (index >= document.parts.size() || index >= poses.size()) {
        return -std::numeric_limits<double>::max();
    }
    const AABB box = transformedBounds(document.parts[index], poses[index]);
    const double area = std::max(1.0, partAreaScore(document.parts[index]));
    const double overlap = intersectionArea(box, target.region.bounds);
    const double fitRatio = overlap / area;
    double score = fitRatio * 1200.0;
    score -= distance(box.center(), target.region.center) * 0.018;

    const bool fitsTarget =
        (box.width() <= target.region.bounds.width() && box.height() <= target.region.bounds.height()) ||
        (box.height() <= target.region.bounds.width() && box.width() <= target.region.bounds.height());
    if (fitsTarget) {
        score += 360.0;
    }
    if (denseSmallMode) {
        const double averageArea = document.parts.empty() ? area : document.totalPartArea() / static_cast<double>(document.parts.size());
        if (area <= averageArea * 0.75) {
            score += 520.0;
        }
    }
    if (map.usedBounds.isValid()) {
        const double eps = 8.0;
        const bool boundary =
            std::abs(box.min.x - map.usedBounds.min.x) <= eps ||
            std::abs(box.max.x - map.usedBounds.max.x) <= eps ||
            std::abs(box.min.y - map.usedBounds.min.y) <= eps ||
            std::abs(box.max.y - map.usedBounds.max.y) <= eps;
        if (boundary) {
            score += 650.0;
        }
    }
    const double aspect = box.height() > 1e-9 ? std::max(box.width(), box.height()) / std::max(1e-9, std::min(box.width(), box.height())) : 1.0;
    if (aspect > 2.0) {
        score += 140.0;
    }
    if (activeBounds.isValid()) {
        AABB merged = activeBounds;
        merged.include(box);
        const double expansion = std::max(0.0, merged.area() - activeBounds.area());
        score -= expansion * 0.006;
        score -= distance(box.center(), activeBounds.center()) * 0.010;
    }
    score += static_cast<double>(((index + 1u) * 1103515245u + placedCount * 2654435761u) & 0xffu) * 0.01;
    return score;
}

size_t chooseNextClusterPart(
    const Document& document,
    const ClusterBeamState& beam,
    const EmptySpaceMap& map,
    const RebuildTarget& target,
    bool denseSmallMode) {
    size_t best = beam.remainingClusterParts.empty() ? static_cast<size_t>(-1) : beam.remainingClusterParts.front();
    double bestScore = -std::numeric_limits<double>::max();
    const std::vector<size_t> activeParts = activePartsForClusterState(beam, document.parts.size());
    const AABB activeBounds = partialBoundsFor(document, beam.poses, activeParts);
    for (size_t index : beam.remainingClusterParts) {
        const double score = partReinsertNeedScore(
            document,
            beam.poses,
            index,
            map,
            target,
            denseSmallMode,
            beam.placedClusterParts.size(),
            activeBounds);
        if (score > bestScore) {
            bestScore = score;
            best = index;
        }
    }
    return best;
}

double clusterCoverageFor(
    const Document& document,
    const std::vector<Pose>& poses,
    const std::vector<size_t>& placed,
    const RebuildTarget& target) {
    double movedArea = 0.0;
    double coverage = 0.0;
    for (size_t index : placed) {
        if (index >= document.parts.size() || index >= poses.size()) {
            continue;
        }
        const AABB box = transformedBounds(document.parts[index], poses[index]);
        movedArea += std::max(1.0, partAreaScore(document.parts[index]));
        coverage += intersectionArea(box, target.region.bounds);
    }
    return movedArea > 1e-9 ? coverage / movedArea : 0.0;
}

AABB partialBoundsFor(const Document& document, const std::vector<Pose>& poses, const std::vector<size_t>& activeParts) {
    AABB used;
    for (size_t index : activeParts) {
        if (index < document.parts.size() && index < poses.size()) {
            used.include(transformedBounds(document.parts[index], poses[index]));
        }
    }
    return used;
}

double partialUsedEstimate(const Document& document, const std::vector<Pose>& poses, const std::vector<size_t>& activeParts) {
    const AABB used = partialBoundsFor(document, poses, activeParts);
    return used.isValid() ? used.area() : 0.0;
}

std::vector<size_t> activePartsForClusterState(const ClusterBeamState& state, size_t limit) {
    std::vector<size_t> active = state.fixedParts;
    for (size_t index : state.placedClusterParts) {
        appendUnique(active, index, limit);
    }
    return active;
}

void updateClusterBeamRank(
    const Document& document,
    const EngineSettings& settings,
    ClusterBeamState& state,
    const RebuildTarget& target) {
    const std::vector<size_t> activeParts = activePartsForClusterState(state, document.parts.size());
    state.usedBoundsEstimate = partialUsedEstimate(document, state.poses, activeParts);
    state.emptyRegionCoverage = clusterCoverageFor(document, state.poses, state.placedClusterParts, target);
    state.localRank = nodeRank(document, settings, state.poses, target, state.contactGain, state.placedClusterParts, activeParts) -
        state.emptyRegionCoverage * std::max(1.0, target.region.area) * 8.0 +
        state.averageClearance * 120.0 +
        static_cast<double>(state.restoreFallbacks) * std::max(1000.0, target.region.area * 0.05);
}

std::vector<size_t> checkPartsForClusterState(
    const std::vector<size_t>& fixed,
    const std::vector<size_t>& placed,
    size_t limit) {
    std::vector<size_t> checkParts = fixed;
    for (size_t index : placed) {
        appendUnique(checkParts, index, limit);
    }
    return checkParts;
}

std::vector<ClusterCandidateChoice> generateClusterCandidateChoices(
    const Document& document,
    const EngineSettings& settings,
    const ClusterBeamState& state,
    size_t partIndex,
    const std::vector<size_t>& checkParts,
    const RebuildTarget& target,
    const ClusterBeamParams& params,
    const std::vector<Vec2>& anchors,
    const IContactCandidateProvider& ifpProvider,
    const IContactCandidateProvider& nfpProvider,
    const IContactCandidateProvider& analyticProvider,
    SolverStats& stats,
    bool allowRestoreFallback) {
    std::vector<ClusterCandidateChoice> choices;
    if (partIndex >= document.parts.size() || partIndex >= state.poses.size()) {
        return choices;
    }

    ContactCandidateRequest request;
    request.movingPart = partIndex;
    request.fixedParts = checkParts;
    request.regionAnchors = anchors;
    request.angles = angleSamples(settings, state.poses[partIndex]);
    request.mirrors = mirrorSamples(settings, state.poses[partIndex]);
    request.ownerLimit = params.ownerLimit;
    request.perOwnerPointLimit = params.pointLimit;
    request.candidateLimit = params.candidateLimit;

    std::vector<ContactCandidate> generated;
    auto appendGenerated = [&](std::vector<ContactCandidate>&& incoming) {
        for (const ContactCandidate& candidate : incoming) {
            const bool exists = std::any_of(generated.begin(), generated.end(), [&](const ContactCandidate& existing) {
                return poseChanged(existing.pose, candidate.pose) == false;
            });
            if (!exists) {
                generated.push_back(candidate);
            }
        }
    };

    if (settings.performanceProfile != PerformanceProfile::Fast) {
        ContactCandidateStats ifpStats;
        appendGenerated(ifpProvider.generatePartSheetCandidates(document, settings, state.poses, request, &ifpStats));
        stats.ifpCandidatesGenerated += ifpStats.generated;
        stats.ifpCandidatesValid += ifpStats.valid;
        stats.contactCandidatesRejectedCollision += ifpStats.rejectedCollision;
        stats.contactCandidatesRejectedClearance += ifpStats.rejectedClearance;
        stats.contactCandidatesRejectedSheet += ifpStats.rejectedSheet;

        ContactCandidateStats nfpStats;
        appendGenerated(nfpProvider.generatePartPartCandidates(document, settings, state.poses, request, &nfpStats));
        stats.nfpCandidatesGenerated += nfpStats.generated;
        stats.nfpCandidatesValid += nfpStats.valid;
        stats.nfpCacheHits += nfpStats.cacheHits;
        stats.nfpCacheMisses += nfpStats.cacheMisses;
        stats.contactCandidatesRejectedCollision += nfpStats.rejectedCollision;
        stats.contactCandidatesRejectedClearance += nfpStats.rejectedClearance;
        stats.contactCandidatesRejectedSheet += nfpStats.rejectedSheet;
    }

    if (generated.size() < params.candidatePerPart || settings.performanceProfile == PerformanceProfile::Fast) {
        ContactCandidateStats analyticStats;
        appendGenerated(analyticProvider.generateCandidates(document, settings, state.poses, request, &analyticStats));
        stats.analyticFallbackCandidatesGenerated += analyticStats.generated;
        stats.analyticFallbackCandidatesValid += analyticStats.valid;
        stats.analyticCandidatesGenerated += analyticStats.generated;
        stats.analyticCandidatesValid += analyticStats.valid;
        stats.contactCandidatesRejectedCollision += analyticStats.rejectedCollision;
        stats.contactCandidatesRejectedClearance += analyticStats.rejectedClearance;
        stats.contactCandidatesRejectedSheet += analyticStats.rejectedSheet;
    }

    if (allowRestoreFallback && poseValidAgainstParts(document, settings, state.poses, partIndex, state.poses[partIndex], checkParts)) {
        generated.push_back({state.poses[partIndex], AnalyticContactKind::RegionAnchor, static_cast<size_t>(-1), -1, -600.0});
    }
    if (generated.empty()) {
        return choices;
    }

    const AABB fixedBounds = boundsForParts(document, state.poses, checkParts);
    std::stable_sort(generated.begin(), generated.end(), [&](const ContactCandidate& a, const ContactCandidate& b) {
        const double scoreA = localPlacementScore(document, partIndex, a.pose, fixedBounds, target, a.priority);
        const double scoreB = localPlacementScore(document, partIndex, b.pose, fixedBounds, target, b.priority);
        if (std::abs(scoreA - scoreB) > 1e-9) {
            return scoreA < scoreB;
        }
        return a.priority > b.priority;
    });

    const size_t generationLimit = std::min(generated.size(), params.candidatePerPart * 2u);
    for (size_t i = 0; i < generationLimit && choices.size() < params.candidatePerPart; ++i) {
        ContactCandidate candidate = generated[i];
        if (!poseValidAgainstParts(document, settings, state.poses, partIndex, candidate.pose, checkParts)) {
            continue;
        }
        candidate.pose = contactLockedPose(document, settings, state.poses, partIndex, candidate.pose, checkParts, target, fixedBounds);
        if (!poseValidAgainstParts(document, settings, state.poses, partIndex, candidate.pose, checkParts)) {
            continue;
        }
        const bool restoreFallback =
            candidate.kind == AnalyticContactKind::RegionAnchor &&
            candidate.sourcePart == static_cast<size_t>(-1) &&
            candidate.priority < -500.0;
        choices.push_back({candidate, restoreFallback});
    }
    return choices;
}

bool compactionImproves(const Objective& before, const Objective& after);
std::vector<Pose> shiftedClusterPoses(std::vector<Pose> poses, const std::vector<size_t>& cluster, Vec2 direction, double distance);

Objective polishClusterLeaf(
    const Document& document,
    const EngineSettings& settings,
    const Objective& leaf,
    const std::vector<size_t>& cluster,
    const RebuildTarget& target,
    PenaltySystem& attemptPenalties,
    PenaltySystem& globalPenalties) {
    Objective best = leaf;
    std::vector<Vec2> directions;
    const double hx = [&]() {
        switch (settings.placementStrategy) {
        case PlacementStrategy::BottomRight:
        case PlacementStrategy::TopRight:
        case PlacementStrategy::RightToLeft:
            return 1.0;
        default:
            return -1.0;
        }
    }();
    const double vy = [&]() {
        switch (settings.placementStrategy) {
        case PlacementStrategy::TopLeft:
        case PlacementStrategy::TopRight:
        case PlacementStrategy::TopToBottom:
            return 1.0;
        default:
            return -1.0;
        }
    }();
    directions.push_back({hx, 0.0});
    directions.push_back({0.0, vy});
    directions.push_back({hx, vy});
    const AABB clusterBounds = boundsForParts(document, leaf.state.poses, cluster);
    if (clusterBounds.isValid()) {
        directions.insert(directions.begin(), target.region.center - clusterBounds.center());
    }

    const double maxPush = std::max(8.0, std::min(document.sheet.width, document.sheet.height) * 0.10);
    for (Vec2 direction : directions) {
        if (direction.length() <= 1e-9) {
            continue;
        }
        double low = 0.0;
        double high = maxPush;
        for (int step = 0; step < 14; ++step) {
            const double mid = (low + high) * 0.5;
            Objective trial = evaluateObjective(document, settings, shiftedClusterPoses(best.state.poses, cluster, direction, mid), attemptPenalties, globalPenalties);
            if (trial.state.valid()) {
                low = mid;
                if (compactionImproves(best, trial)) {
                    best = std::move(trial);
                }
            } else {
                high = mid;
            }
        }
    }

    const int passes = document.parts.size() >= 400u ? 1 : 2;
    const double maxSlide = std::max(8.0, std::min(document.sheet.width, document.sheet.height) * 0.08);
    for (int pass = 0; pass < passes; ++pass) {
        bool moved = false;
        for (size_t partIndex : cluster) {
            if (partIndex >= best.state.poses.size()) {
                continue;
            }
            for (Vec2 direction : compactionDirections(document, settings, best.state, partIndex, target)) {
                Pose movedPose;
                if (!slidePoseSafely(document, settings, best.state.poses, partIndex, direction, maxSlide, movedPose)) {
                    continue;
                }
                std::vector<Pose> poses = best.state.poses;
                poses[partIndex] = movedPose;
                Objective trial = evaluateObjective(document, settings, poses, attemptPenalties, globalPenalties);
                if (compactionImproves(best, trial)) {
                    best = std::move(trial);
                    moved = true;
                    break;
                }
            }
        }
        if (!moved) {
            break;
        }
    }
    return best;
}

Objective coordinatedClusterRebuild(
    const Document& document,
    const EngineSettings& settings,
    const Objective& base,
    const std::vector<size_t>& cluster,
    const RebuildTarget& target,
    const EmptySpaceMap& map,
    PenaltySystem& attemptPenalties,
    PenaltySystem& globalPenalties,
    SolverStats& stats) {
    if (cluster.size() < 3u) {
        return base;
    }
    ++stats.coordinatedClusterRebuildAttempts;
    stats.coordinatedClusterSizeTotal += cluster.size();
    stats.averageCoordinatedClusterSize =
        static_cast<double>(stats.coordinatedClusterSizeTotal) /
        static_cast<double>(std::max<size_t>(1, stats.coordinatedClusterRebuildAttempts));
    const double averageArea = document.parts.empty() ? 0.0 : document.totalPartArea() / static_cast<double>(document.parts.size());
    const size_t smallCount = static_cast<size_t>(std::count_if(cluster.begin(), cluster.end(), [&](size_t index) {
        return index < document.parts.size() && partAreaScore(document.parts[index]) <= averageArea * 0.70;
    }));
    const bool denseSmallMode = document.parts.size() >= 80u || smallCount * 2u >= cluster.size();
    if (denseSmallMode) {
        ++stats.denseSmallPartCompactionAttempts;
    }

    const std::vector<Vec2> anchors = targetAnchors(target, map);
    const ClusterBeamParams params = clusterBeamParamsFor(settings, document.parts.size(), cluster.size(), denseSmallMode);
    const std::vector<size_t> fixed = outsideClusterParts(document, base.state.poses, cluster);
    InnerFitCandidateProvider ifpProvider;
    NfpCandidateProvider nfpProvider;
    AnalyticContactCandidateProvider analyticProvider;

    ClusterBeamState root;
    root.poses = base.state.poses;
    root.fixedParts = fixed;
    root.remainingClusterParts = clusterReinsertOrder(document, base.state, cluster, map, target);
    updateClusterBeamRank(document, settings, root, target);

    std::vector<ClusterBeamState> beam{std::move(root)};
    std::vector<ClusterBeamState> leaves;
    leaves.reserve(params.validLeafLimit);
    const size_t maxDepth = std::max<size_t>(1u, params.maxDepth);

    for (size_t depth = 0; depth < maxDepth && !beam.empty(); ++depth) {
        std::vector<ClusterBeamState> nextBeam;
        ClusterBeamParams depthParams = params;
        size_t expandBudget = std::max<size_t>(3u, params.beamWidth / 3u);
        if (settings.performanceProfile == PerformanceProfile::Maximum) {
            if (depth >= 8u) {
                depthParams.candidatePerPart = std::min<size_t>(depthParams.candidatePerPart, denseSmallMode ? 12u : 10u);
                depthParams.candidateLimit = std::min<size_t>(depthParams.candidateLimit, denseSmallMode ? 30u : 24u);
                depthParams.ownerLimit = std::min<size_t>(depthParams.ownerLimit, 8u);
                expandBudget = std::max<size_t>(4u, std::min<size_t>(expandBudget, 6u));
            }
            if (depth >= 16u) {
                depthParams.candidatePerPart = std::min<size_t>(depthParams.candidatePerPart, denseSmallMode ? 8u : 6u);
                depthParams.candidateLimit = std::min<size_t>(depthParams.candidateLimit, denseSmallMode ? 20u : 16u);
                depthParams.ownerLimit = std::min<size_t>(depthParams.ownerLimit, 5u);
                expandBudget = std::max<size_t>(4u, std::min<size_t>(expandBudget, 5u));
            }
        } else {
            if (depth >= 6u) {
                depthParams.candidatePerPart = std::min<size_t>(depthParams.candidatePerPart, 6u);
                depthParams.candidateLimit = std::min<size_t>(depthParams.candidateLimit, 18u);
                depthParams.ownerLimit = std::min<size_t>(depthParams.ownerLimit, 6u);
                depthParams.pointLimit = 1u;
                expandBudget = std::min<size_t>(expandBudget, 4u);
            }
            if (depth >= 12u) {
                depthParams.candidatePerPart = std::min<size_t>(depthParams.candidatePerPart, 4u);
                depthParams.candidateLimit = std::min<size_t>(depthParams.candidateLimit, 12u);
                depthParams.ownerLimit = std::min<size_t>(depthParams.ownerLimit, 4u);
                expandBudget = std::min<size_t>(expandBudget, 3u);
            }
        }
        const size_t statesToExpand = std::min(beam.size(), expandBudget);
        std::stable_sort(beam.begin(), beam.end(), [](const ClusterBeamState& a, const ClusterBeamState& b) {
            return a.localRank < b.localRank;
        });

        for (size_t stateIndex = 0; stateIndex < statesToExpand; ++stateIndex) {
            const ClusterBeamState& state = beam[stateIndex];
            if (state.remainingClusterParts.empty()) {
                leaves.push_back(state);
                continue;
            }

            const size_t partIndex = chooseNextClusterPart(document, state, map, target, denseSmallMode);
            if (partIndex == static_cast<size_t>(-1)) {
                leaves.push_back(state);
                continue;
            }

            const std::vector<size_t> checkParts = checkPartsForClusterState(state.fixedParts, state.placedClusterParts, document.parts.size());
            const bool allowRestoreFallback = depth + 4u >= maxDepth;
            std::vector<ClusterCandidateChoice> choices = generateClusterCandidateChoices(
                document,
                settings,
                state,
                partIndex,
                checkParts,
                target,
                depthParams,
                anchors,
                ifpProvider,
                nfpProvider,
                analyticProvider,
                stats,
                allowRestoreFallback);

            if (choices.empty()) {
                ClusterBeamState restored = state;
                appendUnique(restored.placedClusterParts, partIndex, cluster.size());
                removePartIndex(restored.remainingClusterParts, partIndex);
                ++restored.restoreFallbacks;
                ++stats.clusterBeamRestoreFallbackCount;
                updateClusterBeamRank(document, settings, restored, target);
                ++stats.clusterBeamStatesGenerated;
                nextBeam.push_back(std::move(restored));
                continue;
            }

            for (const ClusterCandidateChoice& choice : choices) {
                ClusterBeamState child = state;
                child.poses[partIndex] = choice.candidate.pose;
                appendUnique(child.placedClusterParts, partIndex, cluster.size());
                removePartIndex(child.remainingClusterParts, partIndex);
                child.contactGain += std::max(0.0, choice.candidate.priority);
                if (isIfpCandidateKind(choice.candidate.kind)) {
                    ++stats.ifpCandidatesAccepted;
                } else if (isNfpCandidateKind(choice.candidate.kind)) {
                    ++stats.nfpCandidatesAccepted;
                } else if (!choice.restoreFallback) {
                    ++stats.analyticCandidatesAccepted;
                }
                if (choice.restoreFallback) {
                    ++child.restoreFallbacks;
                    ++stats.clusterBeamRestoreFallbackCount;
                }
                updateClusterBeamRank(document, settings, child, target);
                if (choice.restoreFallback) {
                    child.localRank += 50000.0;
                }
                ++stats.clusterBeamStatesGenerated;
                nextBeam.push_back(std::move(child));
            }
        }

        if (nextBeam.empty()) {
            break;
        }
        std::stable_sort(nextBeam.begin(), nextBeam.end(), [](const ClusterBeamState& a, const ClusterBeamState& b) {
            return a.localRank < b.localRank;
        });
        if (nextBeam.size() > params.beamWidth) {
            nextBeam.resize(params.beamWidth);
        }
        stats.clusterBeamStatesKept += nextBeam.size();
        for (const ClusterBeamState& state : nextBeam) {
            if (state.remainingClusterParts.empty() || depth + 1u >= maxDepth) {
                leaves.push_back(state);
                if (leaves.size() >= params.validLeafLimit) {
                    break;
                }
            }
        }
        beam = std::move(nextBeam);
        if (leaves.size() >= params.validLeafLimit) {
            break;
        }
    }

    for (const ClusterBeamState& state : beam) {
        if (leaves.size() >= params.validLeafLimit) {
            break;
        }
        leaves.push_back(state);
    }

    Objective best = base;
    double bestRank = std::numeric_limits<double>::max();
    const size_t leafEvalLimit = std::min(leaves.size(), params.validLeafLimit);
    std::stable_sort(leaves.begin(), leaves.end(), [](const ClusterBeamState& a, const ClusterBeamState& b) {
        return a.localRank < b.localRank;
    });
    for (size_t i = 0; i < leafEvalLimit; ++i) {
        const double restoreRatio = leaves[i].placedClusterParts.empty()
            ? 1.0
            : static_cast<double>(leaves[i].restoreFallbacks) / static_cast<double>(leaves[i].placedClusterParts.size());
        if (restoreRatio > 0.50) {
            continue;
        }
        Objective candidate = evaluateObjective(document, settings, leaves[i].poses, attemptPenalties, globalPenalties);
        if (!candidate.state.valid()) {
            continue;
        }
        candidate = polishClusterLeaf(document, settings, candidate, cluster, target, attemptPenalties, globalPenalties);
        if (!candidate.state.valid()) {
            continue;
        }
        ++stats.clusterBeamLeaves;
        stats.clusterBeamDepthTotal += leaves[i].placedClusterParts.size();
        stats.clusterBeamAverageDepth =
            static_cast<double>(stats.clusterBeamDepthTotal) /
            static_cast<double>(std::max<size_t>(1u, stats.clusterBeamLeaves));
        const double rank = leafObjectiveRank(base, candidate) + leaves[i].localRank * 0.001 + restoreRatio * 100000.0;
        if (compactionImproves(base, candidate) && rank < bestRank) {
            bestRank = rank;
            best = std::move(candidate);
        }
    }
    if (stats.clusterBeamStatesGenerated > 0) {
        stats.clusterBeamRestoreFallbackRatio =
            static_cast<double>(stats.clusterBeamRestoreFallbackCount) /
            static_cast<double>(stats.clusterBeamStatesGenerated);
    }

    if (compactionImproves(base, best)) {
        ++stats.coordinatedClusterRebuildAccepted;
        ++stats.clusterBeamAccepted;
        if (denseSmallMode) {
            ++stats.denseSmallPartCompactionAccepted;
            ++stats.denseClusterBeamAccepted;
        }
        return best;
    }
    return base;
}

std::vector<Pose> shiftedClusterPoses(std::vector<Pose> poses, const std::vector<size_t>& cluster, Vec2 direction, double distance) {
    const double len = direction.length();
    if (len <= 1e-9) {
        return poses;
    }
    direction = direction / len;
    for (size_t index : cluster) {
        if (index < poses.size()) {
            poses[index].x += direction.x * distance;
            poses[index].y += direction.y * distance;
        }
    }
    return poses;
}

Objective coordinatedClusterPush(
    const Document& document,
    const EngineSettings& settings,
    const Objective& base,
    const std::vector<size_t>& cluster,
    const RebuildTarget& target,
    PenaltySystem& attemptPenalties,
    PenaltySystem& globalPenalties,
    SolverStats& stats) {
    if (cluster.empty()) {
        return base;
    }
    std::vector<Vec2> directions;
    const double hx = [&]() {
        switch (settings.placementStrategy) {
        case PlacementStrategy::BottomRight:
        case PlacementStrategy::TopRight:
        case PlacementStrategy::RightToLeft:
            return 1.0;
        default:
            return -1.0;
        }
    }();
    const double vy = [&]() {
        switch (settings.placementStrategy) {
        case PlacementStrategy::TopLeft:
        case PlacementStrategy::TopRight:
        case PlacementStrategy::TopToBottom:
            return 1.0;
        default:
            return -1.0;
        }
    }();
    directions.push_back({hx, 0.0});
    directions.push_back({0.0, vy});
    directions.push_back({hx, vy});
    const AABB clusterBounds = boundsForParts(document, base.state.poses, cluster);
    if (clusterBounds.isValid()) {
        directions.insert(directions.begin(), target.region.center - clusterBounds.center());
        if (settings.placementStrategy == PlacementStrategy::CenterOut ||
            settings.placementStrategy == PlacementStrategy::OutsideIn) {
            const Vec2 sheetCenter{
                document.sheet.origin.x + document.sheet.width * 0.5,
                document.sheet.origin.y + document.sheet.height * 0.5
            };
            directions.insert(directions.begin(), sheetCenter - clusterBounds.center());
        }
    }

    Objective best = base;
    const double maxDistance = std::max(12.0, std::min(document.sheet.width, document.sheet.height) * 0.18);
    for (Vec2 direction : directions) {
        if (direction.length() <= 1e-9) {
            continue;
        }
        double low = 0.0;
        double high = maxDistance;
        double probe = std::min(high, std::max(1.0, high / 12.0));
        while (probe < high) {
            Objective trial = evaluateObjective(document, settings, shiftedClusterPoses(best.state.poses, cluster, direction, probe), attemptPenalties, globalPenalties);
            if (!trial.state.valid()) {
                high = probe;
                break;
            }
            low = probe;
            probe = std::min(high, probe * 1.8);
        }
        if (low <= 1e-6) {
            continue;
        }
        Objective bestDirection = best;
        for (int i = 0; i < 18; ++i) {
            const double mid = (low + high) * 0.5;
            Objective trial = evaluateObjective(document, settings, shiftedClusterPoses(best.state.poses, cluster, direction, mid), attemptPenalties, globalPenalties);
            if (trial.state.valid()) {
                low = mid;
                bestDirection = std::move(trial);
            } else {
                high = mid;
            }
        }
        if (compactionImproves(best, bestDirection)) {
            best = std::move(bestDirection);
            ++stats.coordinatedClusterMotionAccepted;
        }
    }
    return best;
}

bool compactionImproves(const Objective& before, const Objective& after) {
    if (!after.state.valid()) {
        return false;
    }
    const RebuildQualityDelta delta = qualityDelta(before, after);
    if (delta.usedAreaDelta < -1.0) {
        return true;
    }
    if (delta.utilizationDelta > 0.0001) {
        return true;
    }
    const bool dimensionGainWithoutAreaRegression =
        (delta.usedWidthDelta < -0.25 || delta.usedHeightDelta < -0.25) &&
        delta.usedAreaDelta <= 0.0;
    if (dimensionGainWithoutAreaRegression) {
        return true;
    }
    return delta.largestEmptyRegionDelta < -8.0 && delta.usedAreaDelta <= 0.0;
}

Objective compactRebuildObjective(
    const Document& document,
    const EngineSettings& settings,
    const Objective& base,
    const std::vector<size_t>& changed,
    const std::vector<size_t>& subset,
    const RebuildTarget& target,
    const EmptySpaceMap& map,
    PenaltySystem& attemptPenalties,
    PenaltySystem& globalPenalties,
    SolverStats& stats,
    const std::atomic_bool& stopRequested) {
    ++stats.rebuildCompactionAttempts;
    Objective best = base;
    std::vector<size_t> cluster = compactionCluster(document, settings, base.state, changed, subset, target, map);
    if (cluster.empty()) {
        return best;
    }
    ++stats.rebuildCompactionClusters;
    Objective rebuilt = coordinatedClusterRebuild(
        document,
        settings,
        best,
        cluster,
        target,
        map,
        attemptPenalties,
        globalPenalties,
        stats);
    if (compactionImproves(best, rebuilt)) {
        best = std::move(rebuilt);
        ++stats.rebuildCompactionAccepted;
        ++stats.compactionAccepted;
    }
    Objective pushed = coordinatedClusterPush(
        document,
        settings,
        best,
        cluster,
        target,
        attemptPenalties,
        globalPenalties,
        stats);
    if (compactionImproves(best, pushed)) {
        best = std::move(pushed);
        ++stats.rebuildCompactionAccepted;
        ++stats.compactionAccepted;
    }
    const int passes = document.parts.size() >= 400 ? 1 : 2;
    const double maxDistance = std::max(12.0, std::min(document.sheet.width, document.sheet.height) * 0.14);
    for (int pass = 0; pass < passes && !stopRequested.load(); ++pass) {
        bool movedInPass = false;
        for (size_t partIndex : cluster) {
            if (stopRequested.load() || partIndex >= best.state.poses.size()) {
                break;
            }
            const std::vector<Vec2> directions = compactionDirections(document, settings, best.state, partIndex, target);
            for (Vec2 direction : directions) {
                Pose movedPose;
                if (!slidePoseSafely(document, settings, best.state.poses, partIndex, direction, maxDistance, movedPose)) {
                    continue;
                }
                std::vector<Pose> poses = best.state.poses;
                poses[partIndex] = movedPose;
                Objective trial = evaluateObjective(document, settings, poses, attemptPenalties, globalPenalties);
                if (compactionImproves(best, trial)) {
                    best = std::move(trial);
                    movedInPass = true;
                    ++stats.rebuildCompactionAccepted;
                    ++stats.compactionAccepted;
                    break;
                }
            }
        }
        if (!movedInPass) {
            break;
        }
    }
    return best;
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

size_t activeContactDepthFor(const EngineSettings& settings, size_t subsetSize, size_t partCount) {
    (void)partCount;
    if (settings.performanceProfile == PerformanceProfile::Maximum) {
        const size_t target = subsetSize >= 50u ? 34u : subsetSize >= 30u ? 24u : 18u;
        return std::min(subsetSize, target);
    }
    if (settings.performanceProfile == PerformanceProfile::Balanced) {
        return std::min<size_t>(subsetSize, subsetSize >= 24u ? 18u : 12u);
    }
    return std::min<size_t>(subsetSize, 5u);
}

DepthBudget depthBudgetFor(const EngineSettings& settings, size_t depth, size_t activeContactDepth, size_t partCount) {
    DepthBudget budget;
    if (depth >= activeContactDepth) {
        budget.expansionLimit = 1;
        budget.partialEvalLimit = 1;
        budget.analytic = false;
        return budget;
    }
    budget.analytic = true;
    budget.removeUnplacedSubset = false;
    const bool maximum = settings.performanceProfile == PerformanceProfile::Maximum;
    const double t = activeContactDepth > 1u ? static_cast<double>(depth) / static_cast<double>(activeContactDepth - 1u) : 0.0;
    if (maximum) {
        if (t < 0.25) {
            budget.nodesToExpand = partCount >= 400 ? 2u : 3u;
            budget.expansionLimit = partCount >= 400 ? 8u : 8u;
            budget.partialEvalLimit = partCount >= 400 ? 4u : 6u;
            budget.ownerLimit = partCount >= 400 ? 8u : 10u;
            budget.pointLimit = 2u;
            budget.candidateLimit = partCount >= 400 ? 20u : 24u;
        } else if (t < 0.70) {
            budget.nodesToExpand = 2u;
            budget.expansionLimit = partCount >= 400 ? 5u : 6u;
            budget.partialEvalLimit = partCount >= 400 ? 3u : 4u;
            budget.ownerLimit = partCount >= 400 ? 6u : 8u;
            budget.pointLimit = 1u;
            budget.candidateLimit = partCount >= 400 ? 12u : 16u;
        } else {
            budget.nodesToExpand = 1u;
            budget.expansionLimit = 3u;
            budget.partialEvalLimit = 2u;
            budget.ownerLimit = 5u;
            budget.pointLimit = 1u;
            budget.candidateLimit = 10u;
        }
    } else {
        budget.nodesToExpand = 5u;
        budget.expansionLimit = t < 0.45 ? 6u : 4u;
        budget.partialEvalLimit = t < 0.45 ? 3u : 2u;
        budget.ownerLimit = t < 0.45 ? 12u : 8u;
        budget.pointLimit = 2u;
        budget.candidateLimit = t < 0.45 ? 32u : 18u;
    }
    return budget;
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
    Objective bestObjective = currentObjective;
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
        const size_t activeContactDepth = activeContactDepthFor(settings, subset.size(), document.parts.size());
        stats.placementDepthTotal += depthLimit;
        stats.activeContactDepthTotal += activeContactDepth;
        const DepthBudget headlineBudget = depthBudgetFor(settings, 0, activeContactDepth, document.parts.size());
        stats.expansionLimitTotal += headlineBudget.expansionLimit;
        stats.partialEvalLimitTotal += headlineBudget.partialEvalLimit;
        stats.averagePlacementDepth = static_cast<double>(stats.placementDepthTotal) / static_cast<double>(std::max<size_t>(1, stats.destroyAttempts));
        stats.averageActiveContactDepth = static_cast<double>(stats.activeContactDepthTotal) / static_cast<double>(std::max<size_t>(1, stats.destroyAttempts));
        stats.averageExpansionLimit = static_cast<double>(stats.expansionLimitTotal) / static_cast<double>(std::max<size_t>(1, stats.destroyAttempts));
        stats.averagePartialEvalLimit = static_cast<double>(stats.partialEvalLimitTotal) / static_cast<double>(std::max<size_t>(1, stats.destroyAttempts));

        Objective directCompacted = compactRebuildObjective(
            document,
            settings,
            currentObjective,
            subset,
            subset,
            target,
            map,
            attemptPenalties,
            globalPenalties,
            stats,
            stopRequested);
        RebuildQualityDelta directDelta;
        if (betterQuality(directCompacted, bestObjective, &directDelta)) {
            std::vector<size_t> changed = changedPartsAll(directCompacted.state.poses, current.poses);
            if (!changed.empty()) {
                recordAttemptMetrics(stats, currentObjective, directCompacted);
                current = directCompacted.state;
                currentObjective = std::move(directCompacted);
                best = current;
                bestObjective = currentObjective;
                ++stats.destroyAccepted;
                ++stats.acceptedMoves;
                ++stats.activeMoveAcceptedTotal;
                ++stats.destroyBestUpdates;
                ++stats.bestUpdates;
                ++stats.acceptedBetter;
                recordAcceptedReason(stats, classifyGain(directDelta, false));
                if (callback) {
                    ActiveMoveSummary moves;
                    moves.region = changed.size();
                    ConstructiveRebuildProgress progress;
                    progress.current = current;
                    progress.best = best;
                    progress.stats = stats;
                    progress.activeMoves = moves;
                    progress.versionId = ++version;
                    progress.layoutChanged = true;
                    progress.bestUpdated = true;
                    progress.changedParts = changed;
                    progress.rebuildAttempt = attempt + 1u;
                    progress.beamDepth = 0;
                    progress.subsetSize = subset.size();
                    progress.previewTemporary = false;
                    callback(progress);
                }
                continue;
            }
        }

        const size_t beamWidth = settings.performanceProfile == PerformanceProfile::Maximum ? (document.parts.size() >= 400 ? 24u : 32u) : 16u;
        const size_t validLeafLimit = settings.performanceProfile == PerformanceProfile::Maximum ? (document.parts.size() >= 400 ? 100u : 160u) : 80u;
        const std::vector<Vec2> anchors = targetAnchors(target, map);
        const double attemptStartedSeconds = elapsedSeconds(started);
        const double attemptBudget = std::max(0.85, safetyLimit / std::max<double>(1.0, static_cast<double>(attempts)) * 2.8);

        BeamNode root;
        root.poses = current.poses;
        root.rank = nodeRank(document, settings, root.poses, target, 0.0, root.placed);
        std::vector<BeamNode> beam{std::move(root)};
        Objective bestAttempt;
        double bestAttemptRank = std::numeric_limits<double>::max();
        std::vector<LeafCandidate> leafCandidates;
        bool hasAttempt = false;

        for (size_t depth = 0; depth < depthLimit && !beam.empty() && !stopRequested.load(); ++depth) {
            if (elapsedSeconds(started) - attemptStartedSeconds > attemptBudget && hasAttempt) {
                break;
            }
            const size_t partIndex = subset[depth];
            const DepthBudget budget = depthBudgetFor(settings, depth, activeContactDepth, document.parts.size());
            std::vector<BeamNode> expanded;
            expanded.reserve(beamWidth * budget.expansionLimit);
            const size_t nodesToExpand = std::min(budget.nodesToExpand, beam.size());
            for (size_t nodeIndex = 0; nodeIndex < nodesToExpand; ++nodeIndex) {
                const BeamNode& node = beam[nodeIndex];
                std::vector<ContactCandidate> candidates;
                ContactCandidateStats analyticStats;
                if (budget.analytic) {
                    ContactCandidateRequest request;
                    request.movingPart = partIndex;
                    request.fixedParts = fixedPartsFor(document, subset, node.placed, partIndex, budget.removeUnplacedSubset);
                    request.regionAnchors = anchors;
                    request.angles = angleSamples(settings, current.poses[partIndex]);
                    request.mirrors = mirrorSamples(settings, current.poses[partIndex]);
                    request.ownerLimit = budget.ownerLimit;
                    request.perOwnerPointLimit = budget.pointLimit;
                    request.candidateLimit = budget.candidateLimit;
                    candidates = candidateProvider.generateCandidates(document, settings, node.poses, request, &analyticStats);
                }
                if (partIndex < current.poses.size()) {
                    candidates.insert(candidates.begin(), {current.poses[partIndex], AnalyticContactKind::RegionAnchor, static_cast<size_t>(-1), -1, -1000.0});
                }
                if (depth >= activeContactDepth && partIndex < current.poses.size()) {
                    ContactCandidate fallback;
                    fallback.pose = current.poses[partIndex];
                    candidates.clear();
                    candidates.push_back(fallback);
                }
                std::stable_sort(candidates.begin(), candidates.end(), [&](const ContactCandidate& a, const ContactCandidate& b) {
                    const double scoreA = candidateSelectionScore(document, partIndex, a.pose, map.usedBounds, target, a.priority);
                    const double scoreB = candidateSelectionScore(document, partIndex, b.pose, map.usedBounds, target, b.priority);
                    if (std::abs(scoreA - scoreB) > 1e-9) {
                        return scoreA > scoreB;
                    }
                    return a.priority > b.priority;
                });
                candidates.erase(std::remove_if(candidates.begin(), candidates.end(), [&](const ContactCandidate& candidate) {
                    const bool isRestore = partIndex < current.poses.size() && !poseChanged(candidate.pose, current.poses[partIndex]);
                    return !isRestore && !acceptableConstructiveCandidate(document, partIndex, candidate.pose, map.usedBounds, target);
                }), candidates.end());
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
                    if (++used >= budget.expansionLimit) {
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
            const bool shouldEvaluateDepth = depth + 1u >= activeContactDepth ||
                depth + 1u == depthLimit ||
                ((depth + 1u) % 6u == 0u);
            const size_t evalCount = shouldEvaluateDepth ? std::min(budget.partialEvalLimit, expanded.size()) : 0u;
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
                const double attemptRank = leafObjectiveRank(currentObjective, objective);
                const size_t leafKeepLimit = document.parts.size() >= 400 ? 3u : 5u;
                rememberLeafCandidate(leafCandidates, objective, attemptRank, leafKeepLimit);
                if (!hasAttempt || attemptRank < bestAttemptRank) {
                    bestAttempt = std::move(objective);
                    bestAttemptRank = attemptRank;
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
                        progress.rebuildAttempt = attempt + 1u;
                        progress.beamDepth = depth + 1u;
                        progress.subsetSize = subset.size();
                        progress.previewTemporary = true;
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

        if (leafCandidates.empty()) {
            leafCandidates.push_back({bestAttempt, bestAttemptRank});
        }
        Objective selectedAttempt;
        double selectedRank = std::numeric_limits<double>::max();
        bool hasSelectedAttempt = false;
        for (const LeafCandidate& leaf : leafCandidates) {
            std::vector<size_t> leafChanged = changedParts(leaf.objective.state.poses, current.poses, subset);
            if (leafChanged.empty()) {
                continue;
            }
            Objective compacted = compactRebuildObjective(
                document,
                settings,
                leaf.objective,
                leafChanged,
                subset,
                target,
                map,
                attemptPenalties,
                globalPenalties,
                stats,
                stopRequested);
            const double compactedRank = leafObjectiveRank(currentObjective, compacted);
            if (!hasSelectedAttempt || compactedRank < selectedRank) {
                selectedAttempt = std::move(compacted);
                selectedRank = compactedRank;
                hasSelectedAttempt = true;
            }
        }
        if (!hasSelectedAttempt) {
            continue;
        }
        bestAttempt = std::move(selectedAttempt);
        std::vector<size_t> changed = changedPartsAll(bestAttempt.state.poses, current.poses);
        if (changed.empty()) {
            continue;
        }

        recordAttemptMetrics(stats, currentObjective, bestAttempt);
        RebuildQualityDelta bestDelta;
        const bool bestUpdate = betterQuality(bestAttempt, bestObjective, &bestDelta);
        const RebuildQualityDelta currentDelta = qualityDelta(currentObjective, bestAttempt);
        bool accepted = bestUpdate;
        bool temporaryAccepted = false;
        if (!accepted) {
            if (!hasObjectiveGain(currentDelta)) {
                ++stats.destroyTemporaryRejectedNoObjectiveGain;
                ++stats.destroyRejectedWorseAllMetrics;
                ++stats.rejectedByAcceptance;
                continue;
            }
            AdaptiveAcceptanceContext context;
            context.currentScore = currentObjective.value;
            context.candidateScore = bestAttempt.value;
            context.bestScore = bestObjective.value;
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
            } else {
                ++stats.destroyTemporaryAcceptedWithObjectiveGain;
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
            bestObjective = currentObjective;
            ++stats.destroyBestUpdates;
            ++stats.bestUpdates;
            ++stats.acceptedBetter;
            recordAcceptedReason(stats, classifyGain(bestDelta, false));
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
        progress.rebuildAttempt = attempt + 1u;
        progress.beamDepth = activeContactDepth;
        progress.subsetSize = subset.size();
        progress.previewTemporary = !bestUpdate;
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
