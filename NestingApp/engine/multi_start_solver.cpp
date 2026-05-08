#include "engine/multi_start_solver.h"

#include "engine/analytic_contact_candidate.h"
#include "core/math_utils.h"
#include "engine/adaptive_optimizer.h"
#include "engine/aggressive_gap_filler.h"
#include "engine/broadphase.h"
#include "engine/compression.h"
#include "engine/empty_space_map.h"
#include "engine/contact_packing.h"
#include "engine/convergence.h"
#include "engine/constructive_rebuild_engine.h"
#include "engine/destroy_rebuild.h"
#include "engine/escape_search.h"
#include "engine/gap_filling.h"
#include "engine/layout_score.h"
#include "engine/layout_score_components.h"
#include "engine/local_region_repack.h"
#include "engine/overlap_resolver.h"
#include "engine/parallel_collision_evaluator.h"
#include "engine/pose_sampler.h"
#include "engine/rearrangement.h"
#include "engine/region_repack.h"
#include "engine/small_part_filler.h"
#include "engine/ultra_refinement.h"
#include "engine/worker_pool.h"
#include "geometry/clearance.h"
#include "geometry/collision.h"
#include "geometry/transformed_shape.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <numeric>
#include <random>
#include <atomic>
#include <thread>

namespace nest {
namespace {

using Clock = std::chrono::steady_clock;

struct PlacementChoice {
    double angleRadians = 0.0;
    bool mirrored = false;
    AABB bounds;
};

struct LinearPlacementPlan {
    bool primaryX = true;
    int xDirection = 1;
    int yDirection = 1;
};

double scoreArea(const Part& part) {
    return part.area > 0.0 ? part.area : part.localBounds.area();
}

AABB orientedLocalBounds(const Part& part, double angleRadians, bool mirrored) {
    Pose pose;
    pose.angleRadians = angleRadians;
    pose.mirrored = mirrored;
    const Transform transform = pose.toTransform();
    const AABB& source = part.localBounds;
    const Vec2 corners[] = {
        {source.min.x, source.min.y},
        {source.max.x, source.min.y},
        {source.max.x, source.max.y},
        {source.min.x, source.max.y}
    };
    AABB box;
    for (const Vec2& corner : corners) {
        box.include(transform.apply(corner));
    }
    return box;
}

PlacementChoice choosePlacementChoice(const Part& part, const std::vector<double>& rotations, const std::vector<bool>& mirrors, double remainingPrimarySpan, bool primaryX, unsigned int bias) {
    PlacementChoice best;
    double bestScore = std::numeric_limits<double>::max();
    for (const double angle : rotations) {
        for (const bool mirrored : mirrors) {
            const AABB bounds = orientedLocalBounds(part, angle, mirrored);
            if (!bounds.isValid()) {
                continue;
            }
            const double width = std::max(1.0, bounds.width());
            const double height = std::max(1.0, bounds.height());
            const double primarySpan = primaryX ? width : height;
            const double secondarySpan = primaryX ? height : width;
            const double fitPenalty = primarySpan <= remainingPrimarySpan + 1e-6 ? 0.0 : 1000000.0;
            const double score = fitPenalty + primarySpan + secondarySpan * 0.05 + static_cast<double>(bias % 7u) * 0.0001;
            if (score < bestScore) {
                bestScore = score;
                best.angleRadians = angle;
                best.mirrored = mirrored;
                best.bounds = bounds;
            }
        }
    }
    if (!best.bounds.isValid()) {
        best.bounds = part.localBounds;
    }
    return best;
}

LinearPlacementPlan planForStrategy(PlacementStrategy strategy) {
    switch (strategy) {
    case PlacementStrategy::TopLeft:
        return {true, 1, -1};
    case PlacementStrategy::BottomRight:
    case PlacementStrategy::RightToLeft:
        return {true, -1, 1};
    case PlacementStrategy::TopRight:
        return {true, -1, -1};
    case PlacementStrategy::TopToBottom:
        return {false, 1, -1};
    case PlacementStrategy::BottomToTop:
        return {false, 1, 1};
    default:
        return {true, 1, 1};
    }
}

Pose poseFromCornerAnchor(const PlacementChoice& choice, double x, double y, int xDirection, int yDirection) {
    Pose pose;
    pose.x = xDirection >= 0 ? x - choice.bounds.min.x : x - choice.bounds.max.x;
    pose.y = yDirection >= 0 ? y - choice.bounds.min.y : y - choice.bounds.max.y;
    pose.angleRadians = choice.angleRadians;
    pose.mirrored = choice.mirrored;
    return pose;
}

Pose poseFromCenterAnchor(const PlacementChoice& choice, Vec2 anchor) {
    Pose pose;
    const Vec2 center = choice.bounds.center();
    pose.x = anchor.x - center.x;
    pose.y = anchor.y - center.y;
    pose.angleRadians = choice.angleRadians;
    pose.mirrored = choice.mirrored;
    return pose;
}

double clampLocal(double value, double minValue, double maxValue) {
    return std::max(minValue, std::min(value, maxValue));
}

Vec2 centerOutAnchor(size_t placed, double partSpan, const EngineSettings& settings) {
    const Vec2 center{settings.sheetWidth * 0.5, settings.sheetHeight * 0.5};
    if (placed == 0) {
        return center;
    }
    constexpr double goldenAngle = 2.39996322972865332;
    const double radius = std::sqrt(static_cast<double>(placed)) * (partSpan + settings.partSpacing) * 0.85;
    const double angle = static_cast<double>(placed) * goldenAngle;
    return {
        clampLocal(center.x + std::cos(angle) * radius, settings.margin, settings.sheetWidth - settings.margin),
        clampLocal(center.y + std::sin(angle) * radius, settings.margin, settings.sheetHeight - settings.margin)
    };
}

Vec2 outsideInAnchor(size_t placed, double partSpan, const EngineSettings& settings) {
    const double left = settings.margin;
    const double right = settings.sheetWidth - settings.margin;
    const double low = settings.margin;
    const double high = settings.sheetHeight - settings.margin;
    const size_t layer = placed / 4;
    const double inset = std::min({(right - left) * 0.45, (high - low) * 0.45, static_cast<double>(layer) * (partSpan + settings.partSpacing) * 0.35});
    switch (placed % 4) {
    case 1: return {right - inset, high - inset};
    case 2: return {right - inset, low + inset};
    case 3: return {left + inset, high - inset};
    default: return {left + inset, low + inset};
    }
}

bool anchorPlacement(PlacementStrategy strategy) {
    return strategy == PlacementStrategy::CenterOut || strategy == PlacementStrategy::OutsideIn || strategy == PlacementStrategy::UserPoints;
}

double elapsedSeconds(Clock::time_point started) {
    return std::chrono::duration<double>(Clock::now() - started).count();
}

double ultraReserveSeconds(const EngineSettings& settings, double totalLimit) {
    switch (settings.performanceProfile) {
    case PerformanceProfile::Fast:
        return std::min(0.35, std::max(0.05, totalLimit * 0.06));
    case PerformanceProfile::Maximum:
        return std::min(4.0, std::max(0.20, totalLimit * 0.22));
    case PerformanceProfile::Balanced:
    default:
        return std::min(1.5, std::max(0.10, totalLimit * 0.15));
    }
}

size_t inflightMultiplier(PerformanceProfile profile) {
    switch (profile) {
    case PerformanceProfile::Fast:
        return 1;
    case PerformanceProfile::Maximum:
        return 2;
    case PerformanceProfile::Balanced:
    default:
        return 1;
    }
}

int maxAttemptsForProfile(const EngineSettings& settings, size_t workerCount, size_t partCount) {
    const int partFactor = static_cast<int>(std::max<size_t>(1, partCount / 100));
    const double timeScale = std::max(0.25, effectiveSafetyTimeLimitSeconds(settings, partCount) / 5.0);
    switch (settings.performanceProfile) {
    case PerformanceProfile::Fast:
        return std::max(1, static_cast<int>(std::min<double>(8.0 * timeScale, static_cast<double>(workerCount + partFactor))));
    case PerformanceProfile::Maximum:
        return std::max(1, static_cast<int>(std::min<double>(48.0 * timeScale, static_cast<double>(workerCount * 4 + partFactor * 4))));
    case PerformanceProfile::Balanced:
    default:
        return std::max(1, static_cast<int>(std::min<double>(24.0 * timeScale, static_cast<double>(workerCount * 2 + partFactor * 2))));
    }
}

void mergeStats(SolverStats& target, const SolverStats& source) {
    target.evaluatedCandidates += source.evaluatedCandidates;
    target.acceptedMoves += source.acceptedMoves;
    target.rejectedCollision += source.rejectedCollision;
    target.rejectedSpacing += source.rejectedSpacing;
    target.rejectedSheet += source.rejectedSheet;
    target.attemptsStarted += source.attemptsStarted;
    target.attemptsCompleted += source.attemptsCompleted;
    target.bestUpdates += source.bestUpdates;
    target.gapAccepted += source.gapAccepted;
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
    target.ultraAccepted += source.ultraAccepted;
    target.compactionAccepted += source.compactionAccepted;
    target.frontierCandidates += source.frontierCandidates;
    target.smallFillerAccepted += source.smallFillerAccepted;
    target.regionRepackAccepted += source.regionRepackAccepted;
    target.emptySpaceArea = std::max(target.emptySpaceArea, source.emptySpaceArea);
    target.largestEmptyRegionArea = std::max(target.largestEmptyRegionArea, source.largestEmptyRegionArea);
    target.fillableGapCount = std::max(target.fillableGapCount, source.fillableGapCount);
    target.contactCount = std::max(target.contactCount, source.contactCount);
    target.averageClearance = std::max(target.averageClearance, source.averageClearance);
    target.slideToContactAccepted += source.slideToContactAccepted;
    target.aggressiveGapAccepted += source.aggressiveGapAccepted;
    target.localRegionRepackAttempts += source.localRegionRepackAttempts;
    target.localRegionRepackAccepted += source.localRegionRepackAccepted;
    target.localRegionRepackSubsets += source.localRegionRepackSubsets;
    target.localRegionRepackCandidatesGenerated += source.localRegionRepackCandidatesGenerated;
    target.localRegionRepackValidCandidates += source.localRegionRepackValidCandidates;
    target.localRegionRepackNoCandidate += source.localRegionRepackNoCandidate;
    target.localRegionRepackCollisionReject += source.localRegionRepackCollisionReject;
    target.localRegionRepackClearanceReject += source.localRegionRepackClearanceReject;
    target.localRegionRepackSheetReject += source.localRegionRepackSheetReject;
    target.localRegionRepackScoreReject += source.localRegionRepackScoreReject;
    target.localRegionRepackBeamPruned += source.localRegionRepackBeamPruned;
    target.localRegionRepackFullValidationReject += source.localRegionRepackFullValidationReject;
    target.localRegionRepackMaxCandidatesForPart = std::max(target.localRegionRepackMaxCandidatesForPart, source.localRegionRepackMaxCandidatesForPart);
    target.analyticCandidatesGenerated += source.analyticCandidatesGenerated;
    target.analyticCandidatesValid += source.analyticCandidatesValid;
    target.analyticFallbackCandidatesGenerated += source.analyticFallbackCandidatesGenerated;
    target.analyticFallbackCandidatesValid += source.analyticFallbackCandidatesValid;
    target.analyticCandidatesAccepted += source.analyticCandidatesAccepted;
    target.nfpCandidatesGenerated += source.nfpCandidatesGenerated;
    target.nfpCandidatesValid += source.nfpCandidatesValid;
    target.nfpCandidatesAccepted += source.nfpCandidatesAccepted;
    target.ifpCandidatesGenerated += source.ifpCandidatesGenerated;
    target.ifpCandidatesValid += source.ifpCandidatesValid;
    target.ifpCandidatesAccepted += source.ifpCandidatesAccepted;
    target.nfpCacheHits += source.nfpCacheHits;
    target.nfpCacheMisses += source.nfpCacheMisses;
    target.contactCandidatesRejectedCollision += source.contactCandidatesRejectedCollision;
    target.contactCandidatesRejectedClearance += source.contactCandidatesRejectedClearance;
    target.contactCandidatesRejectedSheet += source.contactCandidatesRejectedSheet;
    target.contactCandidatesRejectedScore += source.contactCandidatesRejectedScore;
    target.contourContactAccepted += source.contourContactAccepted;
    target.activeMoveAcceptedTotal += source.activeMoveAcceptedTotal;
    target.acceptedBetter += source.acceptedBetter;
    target.acceptedTemporary += source.acceptedTemporary;
    target.rejectedByScore += source.rejectedByScore;
    target.rejectedByAcceptance += source.rejectedByAcceptance;
    target.destroyAttempts += source.destroyAttempts;
    target.destroyAccepted += source.destroyAccepted;
    target.destroyTemporaryAccepted += source.destroyTemporaryAccepted;
    target.destroyTemporaryAcceptedWithObjectiveGain += source.destroyTemporaryAcceptedWithObjectiveGain;
    target.destroyTemporaryRejectedNoObjectiveGain += source.destroyTemporaryRejectedNoObjectiveGain;
    target.destroyBestUpdates += source.destroyBestUpdates;
    target.destroyRejectedInvalid += source.destroyRejectedInvalid;
    target.destroyRejectedWorseAllMetrics += source.destroyRejectedWorseAllMetrics;
    target.destroyAcceptedReducedUsedBounds += source.destroyAcceptedReducedUsedBounds;
    target.destroyAcceptedReducedLargestEmptyRegion += source.destroyAcceptedReducedLargestEmptyRegion;
    target.destroyAcceptedReducedTotalEmptyArea += source.destroyAcceptedReducedTotalEmptyArea;
    target.destroyAcceptedIncreasedContactWithGapReduction += source.destroyAcceptedIncreasedContactWithGapReduction;
    target.rebuildCompactionAttempts += source.rebuildCompactionAttempts;
    target.rebuildCompactionClusters += source.rebuildCompactionClusters;
    target.rebuildCompactionAccepted += source.rebuildCompactionAccepted;
    target.coordinatedClusterRebuildAttempts += source.coordinatedClusterRebuildAttempts;
    target.coordinatedClusterRebuildAccepted += source.coordinatedClusterRebuildAccepted;
    target.coordinatedClusterMotionAccepted += source.coordinatedClusterMotionAccepted;
    target.denseSmallPartCompactionAttempts += source.denseSmallPartCompactionAttempts;
    target.denseSmallPartCompactionAccepted += source.denseSmallPartCompactionAccepted;
    target.clusterBeamStatesGenerated += source.clusterBeamStatesGenerated;
    target.clusterBeamStatesKept += source.clusterBeamStatesKept;
    target.clusterBeamLeaves += source.clusterBeamLeaves;
    target.clusterBeamAccepted += source.clusterBeamAccepted;
    target.clusterBeamRestoreFallbackCount += source.clusterBeamRestoreFallbackCount;
    target.clusterBeamDepthTotal += source.clusterBeamDepthTotal;
    target.denseClusterBeamAccepted += source.denseClusterBeamAccepted;
    target.coordinatedClusterSizeTotal += source.coordinatedClusterSizeTotal;
    target.destroySubsetTotal += source.destroySubsetTotal;
    target.placementDepthTotal += source.placementDepthTotal;
    target.activeContactDepthTotal += source.activeContactDepthTotal;
    target.expansionLimitTotal += source.expansionLimitTotal;
    target.partialEvalLimitTotal += source.partialEvalLimitTotal;
    target.rebuildPreviewEvents += source.rebuildPreviewEvents;
    target.beamNodesExpanded += source.beamNodesExpanded;
    target.beamValidLeaves += source.beamValidLeaves;
    target.rebuildBeforeUtilization = source.rebuildBeforeUtilization;
    target.rebuildAfterUtilization = source.rebuildAfterUtilization;
    target.rebuildBeforeUsedArea = source.rebuildBeforeUsedArea;
    target.rebuildAfterUsedArea = source.rebuildAfterUsedArea;
    target.rebuildBeforeLargestEmptyRegion = source.rebuildBeforeLargestEmptyRegion;
    target.rebuildAfterLargestEmptyRegion = source.rebuildAfterLargestEmptyRegion;
    target.rebuildBeforeTotalEmptyArea = source.rebuildBeforeTotalEmptyArea;
    target.rebuildAfterTotalEmptyArea = source.rebuildAfterTotalEmptyArea;
    target.rebuildBeforeContactCount = source.rebuildBeforeContactCount;
    target.rebuildAfterContactCount = source.rebuildAfterContactCount;
    target.rebuildBeforeAverageClearance = source.rebuildBeforeAverageClearance;
    target.rebuildAfterAverageClearance = source.rebuildAfterAverageClearance;
    target.rebuildBeforeTowerScore = source.rebuildBeforeTowerScore;
    target.rebuildAfterTowerScore = source.rebuildAfterTowerScore;
    target.bestRebuildUsedAreaReduction = std::max(target.bestRebuildUsedAreaReduction, source.bestRebuildUsedAreaReduction);
    target.bestRebuildUsedWidthReduction = std::max(target.bestRebuildUsedWidthReduction, source.bestRebuildUsedWidthReduction);
    target.bestRebuildUsedHeightReduction = std::max(target.bestRebuildUsedHeightReduction, source.bestRebuildUsedHeightReduction);
    target.bestRebuildUtilizationGain = std::max(target.bestRebuildUtilizationGain, source.bestRebuildUtilizationGain);
    target.bestRebuildLargestEmptyRegionReduction = std::max(target.bestRebuildLargestEmptyRegionReduction, source.bestRebuildLargestEmptyRegionReduction);
    target.bestRebuildTotalEmptyAreaReduction = std::max(target.bestRebuildTotalEmptyAreaReduction, source.bestRebuildTotalEmptyAreaReduction);
    target.bestRebuildContactGain = std::max(target.bestRebuildContactGain, source.bestRebuildContactGain);
    if (target.destroyAttempts > 0) {
        target.averageSubsetSize = static_cast<double>(target.destroySubsetTotal) / static_cast<double>(target.destroyAttempts);
        target.averagePlacementDepth = static_cast<double>(target.placementDepthTotal) / static_cast<double>(target.destroyAttempts);
        target.averageActiveContactDepth = static_cast<double>(target.activeContactDepthTotal) / static_cast<double>(target.destroyAttempts);
        target.averageExpansionLimit = static_cast<double>(target.expansionLimitTotal) / static_cast<double>(target.destroyAttempts);
        target.averagePartialEvalLimit = static_cast<double>(target.partialEvalLimitTotal) / static_cast<double>(target.destroyAttempts);
    }
    if (target.coordinatedClusterRebuildAttempts > 0) {
        target.averageCoordinatedClusterSize =
            static_cast<double>(target.coordinatedClusterSizeTotal) /
            static_cast<double>(target.coordinatedClusterRebuildAttempts);
    }
    if (target.clusterBeamLeaves > 0) {
        target.clusterBeamAverageDepth =
            static_cast<double>(target.clusterBeamDepthTotal) /
            static_cast<double>(target.clusterBeamLeaves);
    }
    if (target.clusterBeamStatesGenerated > 0) {
        target.clusterBeamRestoreFallbackRatio =
            static_cast<double>(target.clusterBeamRestoreFallbackCount) /
            static_cast<double>(target.clusterBeamStatesGenerated);
    }
    const size_t acceptedTotal = target.acceptedMoves;
    const size_t rejectedTotal = target.rejectedByAcceptance + target.rejectedWorseMoves + target.rejectedByScore;
    if (acceptedTotal + rejectedTotal > 0) {
        target.acceptanceRate = static_cast<double>(acceptedTotal) / static_cast<double>(acceptedTotal + rejectedTotal);
    }
    target.towerScore = std::max(target.towerScore, source.towerScore);
    target.layoutSpreadScore = std::max(target.layoutSpreadScore, source.layoutSpreadScore);
    target.unusedSheetRegionScore = std::max(target.unusedSheetRegionScore, source.unusedSheetRegionScore);
    target.lowContactPartCount = std::max(target.lowContactPartCount, source.lowContactPartCount);
    target.qualityFailReason += source.qualityFailReason;
    target.rowBaselineUsed += source.rowBaselineUsed;
    target.contourSeedUsed += source.contourSeedUsed;
    target.rowFallbackUsed += source.rowFallbackUsed;
    target.activeMoveSummary.contact += source.activeMoveSummary.contact;
    target.activeMoveSummary.compression += source.activeMoveSummary.compression;
    target.activeMoveSummary.gap += source.activeMoveSummary.gap;
    target.activeMoveSummary.hole += source.activeMoveSummary.hole;
    target.activeMoveSummary.concavity += source.activeMoveSummary.concavity;
    target.activeMoveSummary.smallPart += source.activeMoveSummary.smallPart;
    target.activeMoveSummary.swap += source.activeMoveSummary.swap;
    target.activeMoveSummary.chain += source.activeMoveSummary.chain;
    target.activeMoveSummary.cluster += source.activeMoveSummary.cluster;
    target.activeMoveSummary.region += source.activeMoveSummary.region;
    target.activeMoveSummary.rotation += source.activeMoveSummary.rotation;
    target.activeMoveSummary.mirror += source.activeMoveSummary.mirror;
    target.activeMoveSummary.escape += source.activeMoveSummary.escape;
    target.activeMoveSummary.frontier += source.activeMoveSummary.frontier;
    target.acceptedMoveSummary.contact += source.acceptedMoveSummary.contact;
    target.acceptedMoveSummary.compression += source.acceptedMoveSummary.compression;
    target.acceptedMoveSummary.gap += source.acceptedMoveSummary.gap;
    target.acceptedMoveSummary.hole += source.acceptedMoveSummary.hole;
    target.acceptedMoveSummary.concavity += source.acceptedMoveSummary.concavity;
    target.acceptedMoveSummary.smallPart += source.acceptedMoveSummary.smallPart;
    target.acceptedMoveSummary.swap += source.acceptedMoveSummary.swap;
    target.acceptedMoveSummary.chain += source.acceptedMoveSummary.chain;
    target.acceptedMoveSummary.cluster += source.acceptedMoveSummary.cluster;
    target.acceptedMoveSummary.region += source.acceptedMoveSummary.region;
    target.acceptedMoveSummary.rotation += source.acceptedMoveSummary.rotation;
    target.acceptedMoveSummary.mirror += source.acceptedMoveSummary.mirror;
    target.acceptedMoveSummary.escape += source.acceptedMoveSummary.escape;
    target.acceptedMoveSummary.frontier += source.acceptedMoveSummary.frontier;
}

void refreshTimingStats(SolverStats& stats, Clock::time_point started) {
    stats.elapsedMs = elapsedSeconds(started) * 1000.0;
    const double elapsed = std::max(0.001, stats.elapsedMs / 1000.0);
    stats.candidatesPerSecond = static_cast<double>(stats.evaluatedCandidates) / elapsed;
}

void refreshQualityStats(const Document& document, const EngineSettings& settings, const LayoutState& state, SolverStats& stats) {
    EmptySpaceAnalyzer analyzer;
    const EmptySpaceMap map = analyzer.analyze(document, settings, state);
    stats.emptySpaceArea = map.totalEmptyArea;
    stats.largestEmptyRegionArea = map.largestRegionArea;
    const double avgSmall = std::max(1.0, document.totalPartArea() / std::max<size_t>(1, document.parts.size()) * 0.35);
    stats.fillableGapCount = map.fillableRegionCount(avgSmall);
    stats.contactCount = static_cast<size_t>(std::llround(std::max(0.0, state.contactReward)));
    const LayoutShapeMetrics shapeMetrics = computeLayoutShapeMetrics(document, settings, map.usedBounds);
    stats.towerScore = shapeMetrics.towerScore;
    stats.layoutSpreadScore = shapeMetrics.layoutSpreadScore;
    stats.unusedSheetRegionScore = shapeMetrics.unusedSheetRegionScore;
    BroadPhase broad;
    const auto nearPairs = broad.findCandidatePairs(document.parts, state.poses, std::max(settings.partSpacing, 25.0));
    double clearanceSum = 0.0;
    size_t clearanceSamples = 0;
    std::vector<size_t> contactByPart(std::min(document.parts.size(), state.poses.size()), 0);
    for (const auto& [a, b] : nearPairs) {
        if (a >= document.parts.size() || b >= document.parts.size() || a >= state.poses.size() || b >= state.poses.size()) {
            continue;
        }
        if (partsCollide(document.parts[a], state.poses[a], document.parts[b], state.poses[b], settings.collisionTolerance)) {
            clearanceSum += 0.0;
            ++clearanceSamples;
            if (a < contactByPart.size()) {
                ++contactByPart[a];
            }
            if (b < contactByPart.size()) {
                ++contactByPart[b];
            }
            continue;
        }
        const ClearanceResult clearance = minimumBoundaryDistance(
            transformPart(document.parts[a], state.poses[a], static_cast<int>(a)),
            transformPart(document.parts[b], state.poses[b], static_cast<int>(b)),
            settings.partSpacing,
            settings.collisionTolerance);
        if (std::isfinite(clearance.minDistance)) {
            clearanceSum += clearance.minDistance;
            ++clearanceSamples;
            if (clearance.minDistance <= settings.partSpacing + 2.5) {
                if (a < contactByPart.size()) {
                    ++contactByPart[a];
                }
                if (b < contactByPart.size()) {
                    ++contactByPart[b];
                }
            }
        }
    }
    stats.averageClearance = clearanceSamples > 0 ? clearanceSum / static_cast<double>(clearanceSamples) : 0.0;
    stats.lowContactPartCount = static_cast<size_t>(std::count(contactByPart.begin(), contactByPart.end(), size_t{0}));
}

bool posesDiffer(const std::vector<Pose>& a, const std::vector<Pose>& b) {
    if (a.size() != b.size()) {
        return true;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::abs(a[i].x - b[i].x) > 1e-9 ||
            std::abs(a[i].y - b[i].y) > 1e-9 ||
            std::abs(a[i].angleRadians - b[i].angleRadians) > 1e-9 ||
            a[i].mirrored != b[i].mirrored) {
            return true;
        }
    }
    return false;
}

size_t usableRingPointCount(const std::vector<Vec2>& points) {
    if (points.size() > 2 && almostEqual(points.front(), points.back(), 1e-9)) {
        return points.size() - 1;
    }
    return points.size();
}

std::vector<Vec2> sampleTransformedBoundaryPoints(const TransformedPart& part, size_t limit) {
    std::vector<Vec2> points;
    if (limit == 0) {
        return points;
    }
    for (const TransformedRing& ring : part.rings) {
        const size_t count = usableRingPointCount(ring.points);
        if (count == 0) {
            continue;
        }
        const size_t stride = std::max<size_t>(1, count / std::max<size_t>(1, limit / std::max<size_t>(1, part.rings.size())));
        for (size_t i = 0; i < count && points.size() < limit; i += stride) {
            points.push_back(ring.points[i]);
        }
    }
    return points;
}

std::vector<Vec2> sampleLocalBoundaryPoints(const Part& part, size_t limit) {
    Pose origin;
    return sampleTransformedBoundaryPoints(transformPart(part, origin), limit);
}

std::vector<size_t> contourSeedOrder(const Document& document, unsigned int seed) {
    std::vector<size_t> sorted(document.parts.size());
    std::iota(sorted.begin(), sorted.end(), 0);
    std::stable_sort(sorted.begin(), sorted.end(), [&](size_t a, size_t b) {
        const double areaA = scoreArea(document.parts[a]);
        const double areaB = scoreArea(document.parts[b]);
        if (std::abs(areaA - areaB) > 1e-9) {
            return areaA > areaB;
        }
        return a < b;
    });

    std::vector<size_t> order;
    order.reserve(sorted.size());
    size_t left = 0;
    size_t right = sorted.empty() ? 0 : sorted.size() - 1;
    bool takeLarge = true;
    while (left <= right && !sorted.empty()) {
        if (takeLarge) {
            order.push_back(sorted[left++]);
        } else {
            order.push_back(sorted[right--]);
        }
        takeLarge = !takeLarge;
    }
    if (seed % 2u == 1u && order.size() > 4) {
        std::rotate(order.begin() + 1, order.begin() + static_cast<std::ptrdiff_t>(order.size() / 3), order.end());
    }
    return order;
}

bool candidateFitsPlaced(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const std::vector<size_t>& placed,
    const std::vector<AABB>& placedBounds,
    size_t partIndex,
    const Pose& pose) {
    const AABB candidateBounds = transformedBounds(document.parts[partIndex], pose);
    if (!isPartInsideSheet(document.parts[partIndex], pose, document.sheet, settings.collisionTolerance) ||
        !partRespectsSheetClearance(document.parts[partIndex], pose, document.sheet, settings.margin, settings.collisionTolerance)) {
        return false;
    }
    const AABB expanded = candidateBounds.expanded(settings.partSpacing + settings.collisionTolerance);
    for (size_t p = 0; p < placed.size(); ++p) {
        const size_t other = placed[p];
        if (p < placedBounds.size() && !expanded.overlaps(placedBounds[p], settings.collisionTolerance)) {
            continue;
        }
        if (partsCollide(document.parts[partIndex], pose, document.parts[other], poses[other], settings.collisionTolerance) ||
            !partsRespectClearance(document.parts[partIndex], pose, document.parts[other], poses[other], settings.partSpacing, settings.collisionTolerance)) {
            return false;
        }
    }
    return true;
}

double contourContactScore(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const std::vector<size_t>& placed,
    const std::vector<AABB>& placedBounds,
    size_t partIndex,
    const Pose& pose) {
    const TransformedPart moving = transformPart(document.parts[partIndex], pose, static_cast<int>(partIndex));
    double contact = 0.0;
    const double window = std::max(0.05, settings.collisionTolerance * 10.0);
    const AABB expanded = moving.bounds.expanded(settings.partSpacing + window);
    for (size_t p = 0; p < placed.size(); ++p) {
        const size_t other = placed[p];
        if (p < placedBounds.size() && !expanded.overlaps(placedBounds[p], window)) {
            continue;
        }
        const TransformedPart placedPart = transformPart(document.parts[other], poses[other], static_cast<int>(other));
        const ClearanceResult distanceResult = minimumBoundaryDistance(moving, placedPart, settings.partSpacing, settings.collisionTolerance);
        if (std::abs(distanceResult.minDistance - settings.partSpacing) <= window) {
            contact += 1.0 + document.parts[other].rings.size() * 0.15;
        }
        for (const TransformedRing& ring : placedPart.rings) {
            if (ring.isHole && pointInRing(ring.points, moving.bounds.center(), settings.collisionTolerance) != PointLocation::Outside) {
                contact += 5.0;
            }
        }
    }
    return contact;
}

Vec2 clampedStrategyAnchor(const EngineSettings& settings, const AABB& local, double xBias, double yBias) {
    const double halfW = local.width() * 0.5;
    const double halfH = local.height() * 0.5;
    const double left = settings.margin + halfW;
    const double right = settings.sheetWidth - settings.margin - halfW;
    const double low = settings.margin + halfH;
    const double high = settings.sheetHeight - settings.margin - halfH;
    return {
        clampLocal(left + (right - left) * xBias, left, right),
        clampLocal(low + (high - low) * yBias, low, high)
    };
}

void appendUniqueAnchor(std::vector<Vec2>& anchors, Vec2 anchor) {
    for (Vec2 existing : anchors) {
        if (distance(existing, anchor) < 1e-6) {
            return;
        }
    }
    anchors.push_back(anchor);
}

std::vector<Vec2> strategySheetAnchors(
    const Document& document,
    const EngineSettings& settings,
    const AABB& local,
    size_t placedOrdinal) {
    const Vec2 bottomLeft = clampedStrategyAnchor(settings, local, 0.0, 0.0);
    const Vec2 topLeft = clampedStrategyAnchor(settings, local, 0.0, 1.0);
    const Vec2 bottomRight = clampedStrategyAnchor(settings, local, 1.0, 0.0);
    const Vec2 topRight = clampedStrategyAnchor(settings, local, 1.0, 1.0);
    const Vec2 center = clampedStrategyAnchor(settings, local, 0.5, 0.5);
    const Vec2 leftMid = clampedStrategyAnchor(settings, local, 0.0, 0.5);
    const Vec2 rightMid = clampedStrategyAnchor(settings, local, 1.0, 0.5);
    const Vec2 bottomMid = clampedStrategyAnchor(settings, local, 0.5, 0.0);
    const Vec2 topMid = clampedStrategyAnchor(settings, local, 0.5, 1.0);

    std::vector<Vec2> anchors;
    if (settings.placementStrategy == PlacementStrategy::UserPoints && !document.sheet.getUserPlacementPoints().empty()) {
        for (Vec2 point : document.sheet.getUserPlacementPoints()) {
            appendUniqueAnchor(anchors, point);
        }
    }

    switch (settings.placementStrategy) {
    case PlacementStrategy::TopLeft:
        appendUniqueAnchor(anchors, topLeft);
        appendUniqueAnchor(anchors, topMid);
        appendUniqueAnchor(anchors, leftMid);
        break;
    case PlacementStrategy::BottomRight:
        appendUniqueAnchor(anchors, bottomRight);
        appendUniqueAnchor(anchors, bottomMid);
        appendUniqueAnchor(anchors, rightMid);
        break;
    case PlacementStrategy::TopRight:
        appendUniqueAnchor(anchors, topRight);
        appendUniqueAnchor(anchors, topMid);
        appendUniqueAnchor(anchors, rightMid);
        break;
    case PlacementStrategy::LeftToRight:
        appendUniqueAnchor(anchors, leftMid);
        appendUniqueAnchor(anchors, bottomLeft);
        appendUniqueAnchor(anchors, topLeft);
        break;
    case PlacementStrategy::RightToLeft:
        appendUniqueAnchor(anchors, rightMid);
        appendUniqueAnchor(anchors, bottomRight);
        appendUniqueAnchor(anchors, topRight);
        break;
    case PlacementStrategy::TopToBottom:
        appendUniqueAnchor(anchors, topMid);
        appendUniqueAnchor(anchors, topLeft);
        appendUniqueAnchor(anchors, topRight);
        break;
    case PlacementStrategy::BottomToTop:
        appendUniqueAnchor(anchors, bottomMid);
        appendUniqueAnchor(anchors, bottomLeft);
        appendUniqueAnchor(anchors, bottomRight);
        break;
    case PlacementStrategy::CenterOut:
        appendUniqueAnchor(anchors, center);
        appendUniqueAnchor(anchors, bottomLeft);
        appendUniqueAnchor(anchors, topRight);
        break;
    case PlacementStrategy::OutsideIn: {
        const Vec2 corners[] = {bottomLeft, topRight, bottomRight, topLeft};
        appendUniqueAnchor(anchors, corners[placedOrdinal % 4u]);
        appendUniqueAnchor(anchors, corners[(placedOrdinal + 1u) % 4u]);
        appendUniqueAnchor(anchors, corners[(placedOrdinal + 2u) % 4u]);
        break;
    }
    case PlacementStrategy::UserPoints:
    case PlacementStrategy::BottomLeft:
    default:
        appendUniqueAnchor(anchors, bottomLeft);
        appendUniqueAnchor(anchors, bottomMid);
        appendUniqueAnchor(anchors, leftMid);
        break;
    }

    appendUniqueAnchor(anchors, center);
    appendUniqueAnchor(anchors, bottomLeft);
    appendUniqueAnchor(anchors, bottomRight);
    appendUniqueAnchor(anchors, topLeft);
    appendUniqueAnchor(anchors, topRight);
    appendUniqueAnchor(anchors, bottomMid);
    appendUniqueAnchor(anchors, topMid);
    appendUniqueAnchor(anchors, leftMid);
    appendUniqueAnchor(anchors, rightMid);
    return anchors;
}

double placementStrategyPenalty(const EngineSettings& settings, const AABB& used) {
    if (!used.isValid()) {
        return 0.0;
    }
    switch (settings.placementStrategy) {
    case PlacementStrategy::TopLeft:
        return used.min.x * 5.0 + (settings.sheetHeight - used.max.y) * 5.0;
    case PlacementStrategy::BottomRight:
    case PlacementStrategy::RightToLeft:
        return (settings.sheetWidth - used.max.x) * 5.0 + used.min.y * 5.0;
    case PlacementStrategy::TopRight:
        return (settings.sheetWidth - used.max.x) * 5.0 + (settings.sheetHeight - used.max.y) * 5.0;
    case PlacementStrategy::TopToBottom:
        return (settings.sheetHeight - used.max.y) * 5.0;
    case PlacementStrategy::BottomToTop:
        return used.min.y * 5.0;
    case PlacementStrategy::CenterOut:
        return distance(used.center(), {settings.sheetWidth * 0.5, settings.sheetHeight * 0.5}) * 2.0;
    case PlacementStrategy::OutsideIn:
        return std::min({used.min.x, used.min.y, settings.sheetWidth - used.max.x, settings.sheetHeight - used.max.y}) * 1.5;
    case PlacementStrategy::LeftToRight:
    case PlacementStrategy::UserPoints:
    case PlacementStrategy::BottomLeft:
    default:
        return used.min.x * 5.0 + used.min.y * 5.0;
    }
}

double contourSeedCandidateScore(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const std::vector<size_t>& placed,
    const std::vector<AABB>& placedBounds,
    size_t partIndex,
    const Pose& pose) {
    AABB used;
    for (size_t other : placed) {
        used.include(transformedBounds(document.parts[other], poses[other]));
    }
    const AABB candidateBounds = transformedBounds(document.parts[partIndex], pose);
    used.include(candidateBounds);
    const double contact = contourContactScore(document, settings, poses, placed, placedBounds, partIndex, pose);
    const Vec2 anchor = strategySheetAnchors(document, settings, candidateBounds, placed.size()).front();
    const double anchorBias = distance(candidateBounds.center(), anchor) * 0.04;
    return used.area() + placementStrategyPenalty(settings, used) + anchorBias - contact * 650.0;
}

void appendUniquePose(std::vector<Pose>& poses, const Pose& pose) {
    for (const Pose& existing : poses) {
        if (std::abs(existing.x - pose.x) < 1e-6 &&
            std::abs(existing.y - pose.y) < 1e-6 &&
            std::abs(existing.angleRadians - pose.angleRadians) < 1e-9 &&
            existing.mirrored == pose.mirrored) {
            return;
        }
    }
    poses.push_back(pose);
}

std::vector<Pose> contourSeedCandidates(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const std::vector<size_t>& placed,
    size_t partIndex,
    const std::vector<double>& rotations,
    const std::vector<bool>& mirrors,
    size_t placedOrdinal,
    unsigned int seed) {
    std::vector<Pose> candidates;
    const Part& part = document.parts[partIndex];
    const Vec2 sheetCenter{settings.sheetWidth * 0.5, settings.sheetHeight * 0.5};
    constexpr double goldenAngle = 2.39996322972865332;
    const bool largeJob = document.parts.size() > 180;
    const size_t orientationLimit = largeJob
        ? (settings.performanceProfile == PerformanceProfile::Maximum ? 3u : 2u)
        : (settings.performanceProfile == PerformanceProfile::Maximum ? 6u : 4u);
    size_t orientationCount = 0;
    for (double angle : rotations) {
        for (bool mirrored : mirrors) {
            if (++orientationCount > orientationLimit) {
                break;
            }
            Pose orientation;
            orientation.angleRadians = angle;
            orientation.mirrored = mirrored;
            const AABB local = transformedBounds(part, orientation);
            const std::vector<Vec2> sheetAnchors = strategySheetAnchors(document, settings, local, placedOrdinal);
            for (Vec2 anchor : sheetAnchors) {
                appendUniquePose(candidates, poseFromCenterAnchor({angle, mirrored, local}, anchor));
            }

            const double radius = std::sqrt(static_cast<double>(placedOrdinal + 1u)) *
                std::max(local.width(), local.height()) * 0.85;
            const size_t spiralLimit = largeJob
                ? (settings.performanceProfile == PerformanceProfile::Maximum ? 6u : 4u)
                : 10u;
            for (size_t s = 0; s < spiralLimit; ++s) {
                const double theta = (static_cast<double>(placedOrdinal + s + seed % 17u) + 1.0) * goldenAngle;
                Vec2 anchor{
                    clampLocal(sheetCenter.x + std::cos(theta) * (radius + static_cast<double>(s) * 7.0), settings.margin + local.width() * 0.5, settings.sheetWidth - settings.margin - local.width() * 0.5),
                    clampLocal(sheetCenter.y + std::sin(theta) * (radius + static_cast<double>(s) * 7.0), settings.margin + local.height() * 0.5, settings.sheetHeight - settings.margin - local.height() * 0.5)
                };
                appendUniquePose(candidates, poseFromCenterAnchor({angle, mirrored, local}, anchor));
            }

            const TransformedPart movingAtOrigin = transformPart(part, orientation, static_cast<int>(partIndex));
            const std::vector<Vec2> movingPoints = sampleTransformedBoundaryPoints(
                movingAtOrigin,
                largeJob ? (settings.performanceProfile == PerformanceProfile::Maximum ? 6u : 4u) : 8u);
            const size_t ownerLimit = largeJob
                ? (settings.performanceProfile == PerformanceProfile::Maximum ? 12u : 5u)
                : (settings.performanceProfile == PerformanceProfile::Maximum ? 16u : 10u);
            for (size_t ownerPos = 0; ownerPos < placed.size() && ownerPos < ownerLimit; ++ownerPos) {
                const size_t owner = placed[placed.size() - ownerPos - 1u];
                const TransformedPart placedPart = transformPart(document.parts[owner], poses[owner], static_cast<int>(owner));
                for (const TransformedRing& ring : placedPart.rings) {
                    const std::vector<Vec2> ownerPoints = sampleTransformedBoundaryPoints(
                        {static_cast<int>(owner), {ring}, ring.bounds},
                        largeJob ? (settings.performanceProfile == PerformanceProfile::Maximum ? (ring.isHole ? 10u : 6u) : 4u) : (ring.isHole ? 10u : 6u));
                    for (Vec2 ownerPoint : ownerPoints) {
                        for (Vec2 movingPoint : movingPoints) {
                            Pose candidate = orientation;
                            candidate.x = ownerPoint.x - movingPoint.x;
                            candidate.y = ownerPoint.y - movingPoint.y;
                            appendUniquePose(candidates, candidate);
                        }
                    }
                    if (ring.isHole) {
                        const std::array<Vec2, 5> holeAnchors{
                            ring.bounds.center(),
                            Vec2{ring.bounds.min.x + local.width() * 0.5, ring.bounds.min.y + local.height() * 0.5},
                            Vec2{ring.bounds.max.x - local.width() * 0.5, ring.bounds.min.y + local.height() * 0.5},
                            Vec2{ring.bounds.min.x + local.width() * 0.5, ring.bounds.max.y - local.height() * 0.5},
                            Vec2{ring.bounds.max.x - local.width() * 0.5, ring.bounds.max.y - local.height() * 0.5}
                        };
                        for (Vec2 anchor : holeAnchors) {
                            appendUniquePose(candidates, poseFromCenterAnchor({angle, mirrored, local}, anchor));
                        }
                    }
                }

                const AABB ownerBox = placedPart.bounds;
                const std::array<Vec2, 8> bboxContactAnchors{
                    Vec2{ownerBox.max.x + settings.partSpacing + local.width() * 0.5, ownerBox.min.y + local.height() * 0.5},
                    Vec2{ownerBox.max.x + settings.partSpacing + local.width() * 0.5, ownerBox.center().y},
                    Vec2{ownerBox.max.x + settings.partSpacing + local.width() * 0.5, ownerBox.max.y - local.height() * 0.5},
                    Vec2{ownerBox.min.x - settings.partSpacing - local.width() * 0.5, ownerBox.center().y},
                    Vec2{ownerBox.center().x, ownerBox.max.y + settings.partSpacing + local.height() * 0.5},
                    Vec2{ownerBox.center().x, ownerBox.min.y - settings.partSpacing - local.height() * 0.5},
                    Vec2{ownerBox.min.x + local.width() * 0.5, ownerBox.max.y + settings.partSpacing + local.height() * 0.5},
                    Vec2{ownerBox.max.x - local.width() * 0.5, ownerBox.min.y - settings.partSpacing - local.height() * 0.5}
                };
                for (Vec2 anchor : bboxContactAnchors) {
                    appendUniquePose(candidates, poseFromCenterAnchor({angle, mirrored, local}, anchor));
                }
            }
        }
        if (orientationCount > orientationLimit) {
            break;
        }
    }
    return candidates;
}

double phaseGuardSeconds(size_t partCount, PerformanceProfile profile, double multiplier) {
    const double perPart = profile == PerformanceProfile::Maximum ? 0.00030 :
        profile == PerformanceProfile::Balanced ? 0.00022 : 0.00015;
    return std::min(0.35, std::max(0.02, static_cast<double>(partCount) * perPart * multiplier));
}

bool hasPhaseBudget(Clock::time_point started, double limitSeconds, size_t partCount, PerformanceProfile profile, double multiplier) {
    return elapsedSeconds(started) + phaseGuardSeconds(partCount, profile, multiplier) < limitSeconds;
}

bool qualityBetterLayout(const LayoutState& candidate, const LayoutState& incumbent) {
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
    const double candidateArea = std::max(1.0, candidate.usedWidth * candidate.usedHeight);
    const double incumbentArea = std::max(1.0, incumbent.usedWidth * incumbent.usedHeight);
    if (candidateArea + std::max(1.0, incumbentArea * 0.001) < incumbentArea) {
        return true;
    }
    AABB candidateBounds;
    AABB incumbentBounds;
    for (size_t i = 0; i < candidate.poses.size(); ++i) {
        candidateBounds.include({candidate.poses[i].x, candidate.poses[i].y});
    }
    for (size_t i = 0; i < incumbent.poses.size(); ++i) {
        incumbentBounds.include({incumbent.poses[i].x, incumbent.poses[i].y});
    }
    const double candidateAspect = candidateBounds.isValid() && candidateBounds.width() > 1e-9 && candidateBounds.height() > 1e-9
        ? std::max(candidateBounds.width() / candidateBounds.height(), candidateBounds.height() / candidateBounds.width())
        : 1.0;
    const double incumbentAspect = incumbentBounds.isValid() && incumbentBounds.width() > 1e-9 && incumbentBounds.height() > 1e-9
        ? std::max(incumbentBounds.width() / incumbentBounds.height(), incumbentBounds.height() / incumbentBounds.width())
        : 1.0;
    if (candidateAspect + 0.50 < incumbentAspect && candidate.utilization + 0.05 >= incumbent.utilization) {
        return true;
    }
    return std::abs(candidate.utilization - incumbent.utilization) <= 1e-6 &&
        candidate.totalScore + 1e-9 < incumbent.totalScore;
}

struct AttemptResult {
    LayoutState state;
    SolverStats stats;
    int attempt = 0;
    PlacementStrategy strategy = PlacementStrategy::BottomLeft;
    unsigned int seed = 0;
};

} // namespace

std::vector<PlacementStrategy> MultiStartSolver::strategySchedule(PlacementStrategy preferred) const {
    std::vector<PlacementStrategy> strategies{preferred};
    const PlacementStrategy all[] = {
        PlacementStrategy::BottomLeft,
        PlacementStrategy::TopLeft,
        PlacementStrategy::BottomRight,
        PlacementStrategy::TopRight,
        PlacementStrategy::CenterOut,
        PlacementStrategy::OutsideIn,
        PlacementStrategy::LeftToRight,
        PlacementStrategy::RightToLeft,
        PlacementStrategy::TopToBottom,
        PlacementStrategy::BottomToTop
    };
    for (PlacementStrategy strategy : all) {
        if (std::find(strategies.begin(), strategies.end(), strategy) == strategies.end()) {
            strategies.push_back(strategy);
        }
    }
    return strategies;
}

std::vector<size_t> MultiStartSolver::partOrder(const Document& document, unsigned int seed, int orderMode) const {
    std::vector<size_t> order(document.parts.size());
    std::iota(order.begin(), order.end(), 0);
    switch (orderMode % 4) {
    case 1:
        std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return document.parts[a].localBounds.width() > document.parts[b].localBounds.width(); });
        break;
    case 2:
        std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return document.parts[a].localBounds.height() > document.parts[b].localBounds.height(); });
        break;
    case 3: {
        std::mt19937 rng(seed);
        std::shuffle(order.begin(), order.end(), rng);
        break;
    }
    default:
        std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return scoreArea(document.parts[a]) > scoreArea(document.parts[b]); });
        break;
    }
    return order;
}

LayoutState MultiStartSolver::rowBaseline(const Document& document, const EngineSettings& settings, PlacementStrategy strategy, unsigned int seed, int orderMode) const {
    LayoutState state;
    state.poses.resize(document.parts.size());
    if (document.parts.empty()) {
        return state;
    }

    PoseSampler sampler;
    const auto rotations = sampler.coarseRotationSamples(settings);
    const auto mirrors = sampler.mirrorSamples(settings);
    const auto order = partOrder(document, seed, orderMode);
    const LinearPlacementPlan plan = planForStrategy(strategy);
    const double leftLimit = settings.margin;
    const double rightLimit = settings.sheetWidth - settings.margin;
    const double lowLimit = settings.margin;
    const double highLimit = settings.sheetHeight - settings.margin;
    double x = plan.xDirection > 0 ? leftLimit : rightLimit;
    double y = plan.yDirection > 0 ? lowLimit : highLimit;
    double rowHeight = 0.0;
    double columnWidth = 0.0;

    for (size_t placed = 0; placed < order.size(); ++placed) {
        const size_t idx = order[placed];
        const Part& part = document.parts[idx];
        const double remaining = plan.primaryX
            ? (plan.xDirection > 0 ? rightLimit - x : x - leftLimit)
            : (plan.yDirection > 0 ? highLimit - y : y - lowLimit);
        PlacementChoice choice = choosePlacementChoice(part, rotations, mirrors, std::max(0.0, remaining), plan.primaryX, seed + static_cast<unsigned int>(placed));
        double width = std::max(1.0, choice.bounds.width());
        double height = std::max(1.0, choice.bounds.height());

        if (anchorPlacement(strategy) && (strategy != PlacementStrategy::UserPoints || !document.sheet.getUserPlacementPoints().empty())) {
            Vec2 anchor;
            if (strategy == PlacementStrategy::OutsideIn) {
                anchor = outsideInAnchor(placed, std::max(width, height), settings);
            } else if (strategy == PlacementStrategy::UserPoints) {
                const auto& anchors = document.sheet.getUserPlacementPoints();
                anchor = anchors[placed % anchors.size()];
            } else {
                anchor = centerOutAnchor(placed, std::max(width, height), settings);
            }
            anchor.x = clampLocal(anchor.x, leftLimit + width * 0.5, rightLimit - width * 0.5);
            anchor.y = clampLocal(anchor.y, lowLimit + height * 0.5, highLimit - height * 0.5);
            state.poses[idx] = poseFromCenterAnchor(choice, anchor);
            continue;
        }

        if (plan.primaryX) {
            const bool wraps = plan.xDirection > 0 ? (x + width > rightLimit && x > leftLimit) : (x - width < leftLimit && x < rightLimit);
            if (wraps) {
                x = plan.xDirection > 0 ? leftLimit : rightLimit;
                y += static_cast<double>(plan.yDirection) * (rowHeight + settings.partSpacing);
                rowHeight = 0.0;
                choice = choosePlacementChoice(part, rotations, mirrors, plan.xDirection > 0 ? rightLimit - x : x - leftLimit, true, seed);
                width = std::max(1.0, choice.bounds.width());
                height = std::max(1.0, choice.bounds.height());
            }
            state.poses[idx] = poseFromCornerAnchor(choice, x, y, plan.xDirection, plan.yDirection);
            x += static_cast<double>(plan.xDirection) * (width + settings.partSpacing);
            rowHeight = std::max(rowHeight, height);
        } else {
            const bool wraps = plan.yDirection > 0 ? (y + height > highLimit && y > lowLimit) : (y - height < lowLimit && y < highLimit);
            if (wraps) {
                y = plan.yDirection > 0 ? lowLimit : highLimit;
                x += static_cast<double>(plan.xDirection) * (columnWidth + settings.partSpacing);
                columnWidth = 0.0;
                choice = choosePlacementChoice(part, rotations, mirrors, plan.yDirection > 0 ? highLimit - y : y - lowLimit, false, seed);
                width = std::max(1.0, choice.bounds.width());
                height = std::max(1.0, choice.bounds.height());
            }
            state.poses[idx] = poseFromCornerAnchor(choice, x, y, plan.xDirection, plan.yDirection);
            y += static_cast<double>(plan.yDirection) * (height + settings.partSpacing);
            columnWidth = std::max(columnWidth, width);
        }
    }

    LayoutScore scorer;
    PenaltySystem penalties;
    return scorer.evaluate(document, settings, state.poses, &penalties);
}

LayoutState MultiStartSolver::contourSeedBaseline(const Document& document, const EngineSettings& settings, unsigned int seed) const {
    LayoutState state;
    state.poses.resize(document.parts.size());
    if (document.parts.empty()) {
        return state;
    }

    PoseSampler sampler;
    std::vector<double> rotations = sampler.coarseRotationSamples(settings);
    if (!settings.allowRotation || settings.rotationMode == RotationMode::None) {
        rotations = {0.0};
    }
    std::vector<bool> mirrors = sampler.mirrorSamples(settings);
    if (!settings.allowMirroring) {
        mirrors = {false};
    }

    const std::vector<size_t> order = contourSeedOrder(document, seed);
    std::vector<size_t> placed;
    std::vector<AABB> placedBounds;
    placed.reserve(order.size());
    placedBounds.reserve(order.size());
    for (size_t ordinal = 0; ordinal < order.size(); ++ordinal) {
        const size_t index = order[ordinal];
        std::vector<Pose> candidates = contourSeedCandidates(
            document,
            settings,
            state.poses,
            placed,
            index,
            rotations,
            mirrors,
            ordinal,
            seed + static_cast<unsigned int>(ordinal * 13u));

        Pose bestPose;
        double bestScore = std::numeric_limits<double>::max();
        bool found = false;
        for (const Pose& candidate : candidates) {
            if (!candidateFitsPlaced(document, settings, state.poses, placed, placedBounds, index, candidate)) {
                continue;
            }
            const double score = contourSeedCandidateScore(document, settings, state.poses, placed, placedBounds, index, candidate);
            if (score < bestScore) {
                bestScore = score;
                bestPose = candidate;
                found = true;
            }
        }
        if (!found) {
            LayoutState failed = state;
            LayoutScore scorer;
            PenaltySystem penalties;
            return scorer.evaluate(document, settings, failed.poses, &penalties);
        }
        state.poses[index] = bestPose;
        placed.push_back(index);
        placedBounds.push_back(transformedBounds(document.parts[index], bestPose));
    }

    LayoutScore scorer;
    PenaltySystem penalties;
    return scorer.evaluate(document, settings, state.poses, &penalties);
}

LayoutState MultiStartSolver::solve(
    const Document& document,
    const EngineSettings& settings,
    const std::atomic_bool& stopRequested,
    SolverProgressCallback callback) const {
    const auto started = Clock::now();
    SolverStats aggregateStats;
    PenaltySystem globalPenalty;

    const unsigned int baseSeed = settings.randomSeed == 0u ? 1u : settings.randomSeed;
    LayoutState best;
    bool foundValidBaseline = false;

    if (settings.performanceProfile != PerformanceProfile::Fast) {
        const int contourAttempts = settings.performanceProfile == PerformanceProfile::Maximum ? 3 : 2;
        for (int attempt = 0; attempt < contourAttempts; ++attempt) {
            LayoutState candidate = contourSeedBaseline(document, settings, baseSeed ^ static_cast<unsigned int>(attempt * 104729 + 31));
            if (candidate.valid() && (!foundValidBaseline || qualityBetterLayout(candidate, best))) {
                if (foundValidBaseline) {
                    ++aggregateStats.bestUpdates;
                }
                best = std::move(candidate);
                foundValidBaseline = true;
            }
        }
        if (foundValidBaseline) {
            ++aggregateStats.contourSeedUsed;
            aggregateStats.activeMoveSummary.contact += document.parts.size();
        }
    }

    if (settings.performanceProfile == PerformanceProfile::Fast || !foundValidBaseline) {
        const auto baselineStrategies = strategySchedule(settings.placementStrategy);
        for (PlacementStrategy strategy : baselineStrategies) {
            for (int orderMode = 0; orderMode < 4; ++orderMode) {
                LayoutState candidate = rowBaseline(document, settings, strategy, baseSeed ^ static_cast<unsigned int>(orderMode * 7919 + 17), orderMode);
                if (candidate.valid() && (!foundValidBaseline || qualityBetterLayout(candidate, best))) {
                    if (foundValidBaseline) {
                        ++aggregateStats.bestUpdates;
                    }
                    best = std::move(candidate);
                    foundValidBaseline = true;
                }
            }
            if (foundValidBaseline && settings.performanceProfile == PerformanceProfile::Fast) {
                break;
            }
        }
        if (foundValidBaseline) {
            if (settings.performanceProfile == PerformanceProfile::Fast) {
                ++aggregateStats.rowBaselineUsed;
            } else {
                ++aggregateStats.rowFallbackUsed;
            }
        }
    }
    if (!foundValidBaseline) {
        best = rowBaseline(document, settings, settings.placementStrategy, baseSeed, 0);
        ++aggregateStats.rowFallbackUsed;
        refreshTimingStats(aggregateStats, started);
        lastStats_ = aggregateStats;
        if (callback) {
            callback({SolverPhase::NoValidLayout, SolverStrategy::Idle, 1.0, best, LayoutState{}, elapsedSeconds(started), aggregateStats});
        }
        return best;
    }
    LayoutState current = best;
    if (callback) {
        SolverProgress progress;
        progress.phase = SolverPhase::InitialPlacement;
        progress.currentStrategy = SolverStrategy::AdaptiveSearch;
        progress.progress = 0.12;
        progress.current = current;
        progress.best = best;
        progress.elapsedSeconds = elapsedSeconds(started);
        progress.stats = aggregateStats;
        progress.versionId = 1;
        progress.layoutChanged = true;
        progress.bestUpdated = true;
        callback(progress);
    }

    aggregateStats.workerCount = 1;
    aggregateStats.attemptsStarted = 1;
    const double totalLimit = effectiveSafetyTimeLimitSeconds(settings, document.parts.size());
    if (!stopRequested.load() && settings.performanceProfile != PerformanceProfile::Fast) {
        ConstructiveRebuildEngine constructive;
        LayoutState constructiveBest = constructive.optimize(document, settings, best, globalPenalty, stopRequested, aggregateStats, [&](const ConstructiveRebuildProgress& event) {
            if (!callback) {
                return;
            }
            SolverProgress solverProgress;
            solverProgress.phase = SolverPhase::Rearrangement;
            solverProgress.currentStrategy = SolverStrategy::RegionRepack;
            solverProgress.progress = std::min(0.78, 0.14 + elapsedSeconds(started) / std::max(0.25, totalLimit) * 0.55);
            solverProgress.current = event.current;
            solverProgress.best = event.best;
            solverProgress.elapsedSeconds = elapsedSeconds(started);
            solverProgress.stats = event.stats;
            solverProgress.activeMoves = event.activeMoves;
            solverProgress.versionId = event.versionId;
            solverProgress.layoutChanged = event.layoutChanged;
            solverProgress.lastMovedPart = event.changedParts.empty() ? kNoPartIndex : event.changedParts.front();
            solverProgress.lastMoveStrategy = SolverStrategy::RegionRepack;
            solverProgress.bestUpdated = event.bestUpdated;
            solverProgress.changedParts = event.changedParts;
            solverProgress.rebuildAttempt = event.rebuildAttempt;
            solverProgress.beamDepth = event.beamDepth;
            solverProgress.subsetSize = event.subsetSize;
            solverProgress.previewTemporary = event.previewTemporary;
            callback(solverProgress);
        });
        if (constructiveBest.valid() && qualityBetterLayout(constructiveBest, best)) {
            best = std::move(constructiveBest);
        }
    }

    LayoutState bestBeforeAdaptive = best;
    AdaptiveUnifiedOptimizer optimizer;
    LayoutState optimized = optimizer.optimize(document, settings, best, globalPenalty, stopRequested, aggregateStats, [&](const AdaptiveProgressEvent& event) {
        if (!callback) {
            return;
        }
        const double progress = std::min(0.96, 0.12 + elapsedSeconds(started) / std::max(0.25, totalLimit) * 0.82);
        SolverProgress solverProgress;
        solverProgress.phase = SolverPhase::Exploration;
        solverProgress.currentStrategy = event.strategy;
        solverProgress.progress = progress;
        solverProgress.current = event.current;
        solverProgress.best = event.best;
        solverProgress.elapsedSeconds = elapsedSeconds(started);
        solverProgress.stats = event.stats;
        solverProgress.activeMoves = event.activeMoves;
        solverProgress.versionId = event.versionId;
        solverProgress.layoutChanged = event.layoutChanged;
        solverProgress.lastMovedPart = event.lastMovedPart;
        solverProgress.lastMoveStrategy = event.lastMoveStrategy;
        solverProgress.bestUpdated = event.bestUpdated;
        callback(solverProgress);
    });
    best = qualityBetterLayout(optimized, bestBeforeAdaptive) ? std::move(optimized) : std::move(bestBeforeAdaptive);

    current = best;
    aggregateStats.attemptsCompleted = 1;
    refreshQualityStats(document, settings, best, aggregateStats);
    refreshTimingStats(aggregateStats, started);
    lastStats_ = aggregateStats;
    return best;
}

} // namespace nest
