#include "engine/multi_start_solver.h"

#include "engine/compression.h"
#include "engine/layout_score.h"
#include "engine/overlap_resolver.h"
#include "engine/parallel_collision_evaluator.h"
#include "engine/pose_sampler.h"
#include "engine/ultra_refinement.h"
#include "engine/worker_pool.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <numeric>
#include <random>
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
    switch (settings.qualityMode) {
    case QualityMode::Fast:
        return std::min(0.35, std::max(0.05, totalLimit * 0.06));
    case QualityMode::MaxQuality:
        return std::min(4.0, std::max(0.20, totalLimit * 0.22));
    case QualityMode::Balanced:
    default:
        return std::min(1.5, std::max(0.10, totalLimit * 0.15));
    }
}

struct AttemptResult {
    LayoutState state;
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
    PenaltySystem globalPenalty;

    LayoutState best = rowBaseline(document, settings, settings.placementStrategy, 1u, 0);
    LayoutState current = best;
    if (callback) {
        callback({SolverPhase::InitialPlacement, 0.12, current, best, elapsedSeconds(started)});
    }

    const auto strategies = strategySchedule(settings.placementStrategy);
    const double totalLimit = std::max(0.25, settings.timeLimitSeconds);
    const double ultraReserve = ultraReserveSeconds(settings, totalLimit);
    const double searchLimit = std::max(0.05, totalLimit - ultraReserve);
    const size_t attemptThreads = std::max<size_t>(1, ParallelCollisionEvaluator::resolveThreadCount(settings));
    WorkerPool attemptPool(attemptThreads);
    std::vector<std::future<AttemptResult>> futures;
    int nextAttempt = 0;

    auto launchAttempt = [&](int attempt) {
        const PlacementStrategy strategy = strategies[static_cast<size_t>(attempt) % strategies.size()];
        const int orderMode = attempt % 4;
        const unsigned int seed = static_cast<unsigned int>(attempt + 1) * 2654435761u;
        const PenaltySystem globalSnapshot = globalPenalty;
        futures.push_back(attemptPool.enqueue([&, strategy, orderMode, seed, attempt, globalSnapshot]() {
            PenaltySystem attemptPenalty;
            OverlapResolver resolver;
            Compression compression;
            LayoutScore localScorer;
            const size_t candidateThreads = std::max<size_t>(1, ParallelCollisionEvaluator::resolveThreadCount(settings) / std::max<size_t>(1, attemptThreads));
            WorkerPool candidatePool(candidateThreads);

            LayoutState state = rowBaseline(document, settings, strategy, seed, orderMode);
            state = resolver.resolve(document, settings, std::move(state), attemptPenalty, candidatePool, stopRequested, seed);
            if (state.collisionCount == 0 && state.invalidPartCount == 0 && !stopRequested.load()) {
                state = compression.compressByScore(document, settings, std::move(state), attemptPenalty);
            }
            state = localScorer.evaluate(document, settings, state.poses, &attemptPenalty, &globalSnapshot, 0.10);
            return AttemptResult{std::move(state), attempt, strategy, seed};
        }));
    };

    while (!stopRequested.load() && futures.size() < attemptThreads && elapsedSeconds(started) < searchLimit) {
        launchAttempt(nextAttempt++);
    }

    while (!futures.empty() && !stopRequested.load()) {
        bool completedAny = false;
        for (auto it = futures.begin(); it != futures.end();) {
            if (it->wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                AttemptResult result = it->get();
                it = futures.erase(it);
                completedAny = true;
                current = result.state;

                for (const CollisionPair& pair : current.collisionPairs) {
                    globalPenalty.observeCollision(pair.a, pair.b);
                }

                if (current.valid() && (!best.valid() || current.totalScore < best.totalScore)) {
                    best = current;
                } else if (!best.valid() && current.totalScore < best.totalScore) {
                    best = current;
                }

                if (callback) {
                    const double progressBase = std::min(0.84, 0.18 + elapsedSeconds(started) / searchLimit * 0.66);
                    callback({current.collisionCount == 0 ? SolverPhase::Compression : SolverPhase::CollisionResolution, progressBase, current, best, elapsedSeconds(started)});
                }
            } else {
                ++it;
            }
        }

        while (!stopRequested.load() && elapsedSeconds(started) < searchLimit && futures.size() < attemptThreads) {
            launchAttempt(nextAttempt++);
        }

        if (!completedAny) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    if (!stopRequested.load()) {
        if (callback) {
            callback({SolverPhase::UltraRefinement, 0.88, best, best, elapsedSeconds(started)});
        }
        UltraRefinement ultraRefinement;
        LayoutState refined = ultraRefinement.refine(document, settings, best, globalPenalty, attemptPool, stopRequested);
        if (refined.valid() && (!best.valid() || refined.totalScore < best.totalScore)) {
            best = refined;
        }
        current = refined.valid() ? refined : best;
        if (callback) {
            callback({SolverPhase::UltraRefinement, 0.94, current, best, elapsedSeconds(started)});
        }
    }
    return best;
}

} // namespace nest
