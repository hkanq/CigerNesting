#include "engine/multi_start_solver.h"

#include "engine/adaptive_optimizer.h"
#include "engine/compression.h"
#include "engine/contact_packing.h"
#include "engine/convergence.h"
#include "engine/escape_search.h"
#include "engine/gap_filling.h"
#include "engine/layout_score.h"
#include "engine/overlap_resolver.h"
#include "engine/parallel_collision_evaluator.h"
#include "engine/pose_sampler.h"
#include "engine/rearrangement.h"
#include "engine/region_repack.h"
#include "engine/small_part_filler.h"
#include "engine/ultra_refinement.h"
#include "engine/worker_pool.h"
#include <algorithm>
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

double clamp(double value, double minValue, double maxValue) {
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
        clamp(center.x + std::cos(angle) * radius, settings.margin, settings.sheetWidth - settings.margin),
        clamp(center.y + std::sin(angle) * radius, settings.margin, settings.sheetHeight - settings.margin)
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
}

void refreshTimingStats(SolverStats& stats, Clock::time_point started) {
    stats.elapsedMs = elapsedSeconds(started) * 1000.0;
    const double elapsed = std::max(0.001, stats.elapsedMs / 1000.0);
    stats.candidatesPerSecond = static_cast<double>(stats.evaluatedCandidates) / elapsed;
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

double phaseGuardSeconds(size_t partCount, PerformanceProfile profile, double multiplier) {
    const double perPart = profile == PerformanceProfile::Maximum ? 0.00030 :
        profile == PerformanceProfile::Balanced ? 0.00022 : 0.00015;
    return std::min(0.35, std::max(0.02, static_cast<double>(partCount) * perPart * multiplier));
}

bool hasPhaseBudget(Clock::time_point started, double limitSeconds, size_t partCount, PerformanceProfile profile, double multiplier) {
    return elapsedSeconds(started) + phaseGuardSeconds(partCount, profile, multiplier) < limitSeconds;
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
            anchor.x = clamp(anchor.x, leftLimit + width * 0.5, rightLimit - width * 0.5);
            anchor.y = clamp(anchor.y, lowLimit + height * 0.5, highLimit - height * 0.5);
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
    const auto baselineStrategies = strategySchedule(settings.placementStrategy);
    for (PlacementStrategy strategy : baselineStrategies) {
        for (int orderMode = 0; orderMode < 4; ++orderMode) {
            LayoutState candidate = rowBaseline(document, settings, strategy, baseSeed ^ static_cast<unsigned int>(orderMode * 7919 + 17), orderMode);
            if (candidate.valid() && (!foundValidBaseline || candidate.totalScore < best.totalScore)) {
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
    if (!foundValidBaseline) {
        best = rowBaseline(document, settings, settings.placementStrategy, baseSeed, 0);
        refreshTimingStats(aggregateStats, started);
        lastStats_ = aggregateStats;
        if (callback) {
            callback({SolverPhase::NoValidLayout, SolverStrategy::Idle, 1.0, best, LayoutState{}, elapsedSeconds(started), aggregateStats});
        }
        return best;
    }
    LayoutState current = best;
    if (callback) {
        callback({SolverPhase::InitialPlacement, SolverStrategy::AdaptiveSearch, 0.12, current, best, elapsedSeconds(started), aggregateStats});
    }

    aggregateStats.workerCount = 1;
    aggregateStats.attemptsStarted = 1;
    AdaptiveUnifiedOptimizer optimizer;
    const double totalLimit = effectiveSafetyTimeLimitSeconds(settings, document.parts.size());
    best = optimizer.optimize(document, settings, std::move(best), globalPenalty, stopRequested, aggregateStats, [&](SolverStrategy strategy, const LayoutState& optimizerCurrent, const LayoutState& optimizerBest, const SolverStats& stats) {
        if (!callback) {
            return;
        }
        const double progress = std::min(0.96, 0.12 + elapsedSeconds(started) / std::max(0.25, totalLimit) * 0.82);
        callback({SolverPhase::Exploration, strategy, progress, optimizerCurrent, optimizerBest, elapsedSeconds(started), stats});
    });
    current = best;
    aggregateStats.attemptsCompleted = 1;
    refreshTimingStats(aggregateStats, started);
    lastStats_ = aggregateStats;
    return best;
}

} // namespace nest
