#include "engine/nesting_engine.h"

#include "engine/broadphase.h"
#include "engine/compression.h"
#include "engine/local_search.h"
#include "engine/parallel_collision_evaluator.h"
#include "engine/penalty_system.h"
#include "engine/pose_sampler.h"
#include "engine/worker_pool.h"
#include "geometry/collision.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>

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

struct ValidationSummary {
    CollisionReport collisionReport;
    size_t spacingFailureCount = 0;
    size_t invalidPartCount = 0;
    size_t sheetMarginFailureCount = 0;

    size_t validationFailureCount() const {
        return collisionReport.collisionCount + spacingFailureCount + invalidPartCount;
    }
};

double scoreArea(const Part& part) {
    return part.area > 0.0 ? part.area : part.localBounds.area();
}

double elapsedSecondsSince(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}

AABB orientedLocalBounds(const Part& part, double angleRadians, bool mirrored) {
    if (!part.localBounds.isValid()) {
        return {};
    }

    Pose pose;
    pose.angleRadians = angleRadians;
    pose.mirrored = mirrored;
    const Transform transform = pose.toTransform();
    const AABB& source = part.localBounds;
    const std::array<Vec2, 4> corners{
        Vec2{source.min.x, source.min.y},
        Vec2{source.max.x, source.min.y},
        Vec2{source.max.x, source.max.y},
        Vec2{source.min.x, source.max.y}
    };

    AABB box;
    for (const auto& corner : corners) {
        box.include(transform.apply(corner));
    }
    return box;
}

PlacementChoice choosePlacementChoice(
    const Part& part,
    const std::vector<double>& rotationSamples,
    const std::vector<bool>& mirrorSamples,
    double remainingPrimarySpan,
    bool primaryX) {
    PlacementChoice best;
    double bestScore = std::numeric_limits<double>::max();

    for (const double angle : rotationSamples) {
        for (const bool mirrored : mirrorSamples) {
            const AABB bounds = orientedLocalBounds(part, angle, mirrored);
            if (!bounds.isValid()) {
                continue;
            }
            const double width = std::max(1.0, bounds.width());
            const double height = std::max(1.0, bounds.height());
            const double primarySpan = primaryX ? width : height;
            const double secondarySpan = primaryX ? height : width;
            const bool fitsCurrentRow = primarySpan <= remainingPrimarySpan + 1e-6;
            const double fitPenalty = fitsCurrentRow ? 0.0 : 1000000.0;
            const double score = fitPenalty + primarySpan + secondarySpan * 0.05;
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
    case PlacementStrategy::BottomLeft:
    case PlacementStrategy::LeftToRight:
    case PlacementStrategy::CenterOut:
    case PlacementStrategy::OutsideIn:
    case PlacementStrategy::UserPoints:
    default:
        return {true, 1, 1};
    }
}

bool usesAnchorPlacement(PlacementStrategy strategy) {
    return strategy == PlacementStrategy::CenterOut ||
        strategy == PlacementStrategy::OutsideIn ||
        strategy == PlacementStrategy::UserPoints;
}

double clampToSheet(double value, double minValue, double maxValue) {
    return std::max(minValue, std::min(value, maxValue));
}

Pose poseFromCornerAnchor(const PlacementChoice& choice, double x, double y, int xDirection, int yDirection) {
    Pose pose;
    pose.x = (xDirection >= 0) ? x - choice.bounds.min.x : x - choice.bounds.max.x;
    pose.y = (yDirection >= 0) ? y - choice.bounds.min.y : y - choice.bounds.max.y;
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

Vec2 centerOutAnchor(size_t placed, double partSpan, const EngineSettings& settings) {
    const Vec2 center{settings.sheetWidth * 0.5, settings.sheetHeight * 0.5};
    if (placed == 0) {
        return center;
    }

    constexpr double goldenAngle = 2.39996322972865332;
    const double radius = std::sqrt(static_cast<double>(placed)) * (partSpan + settings.partSpacing) * 0.85;
    const double angle = static_cast<double>(placed) * goldenAngle;
    return {
        clampToSheet(center.x + std::cos(angle) * radius, settings.margin, settings.sheetWidth - settings.margin),
        clampToSheet(center.y + std::sin(angle) * radius, settings.margin, settings.sheetHeight - settings.margin)
    };
}

Vec2 outsideInAnchor(size_t placed, double partSpan, const EngineSettings& settings) {
    const double left = settings.margin;
    const double right = settings.sheetWidth - settings.margin;
    const double low = settings.margin;
    const double high = settings.sheetHeight - settings.margin;
    const size_t layer = placed / 4;
    const double inset = std::min({(right - left) * 0.45, (high - low) * 0.45, static_cast<double>(layer) * (partSpan + settings.partSpacing) * 0.35});
    const double x0 = left + inset;
    const double x1 = right - inset;
    const double y0 = low + inset;
    const double y1 = high - inset;

    switch (placed % 4) {
    case 1:
        return {x1, y1};
    case 2:
        return {x1, y0};
    case 3:
        return {x0, y1};
    default:
        return {x0, y0};
    }
}

CollisionReport evaluateLayout(
    const Document& document,
    const std::vector<Pose>& poses,
    const EngineSettings& settings,
    ParallelCollisionEvaluator& evaluator,
    WorkerPool& workerPool,
    size_t workerThreadCount) {
    BroadPhase broad;
    const auto pairs = broad.findCandidatePairs(document.parts, poses, settings.partSpacing);
    return evaluator.evaluate(document.parts, poses, pairs, settings.collisionTolerance, workerPool, workerThreadCount);
}

double layoutUtilization(const Document& document, const std::vector<Pose>& poses, const EngineSettings& settings) {
    if (document.parts.empty() || poses.empty()) {
        return 0.0;
    }

    AABB used;
    const size_t count = std::min(document.parts.size(), poses.size());
    for (size_t i = 0; i < count; ++i) {
        used.include(transformedBounds(document.parts[i], poses[i]));
    }

    const double partArea = document.totalPartArea();
    const double sheetArea = std::max(1.0, (settings.sheetWidth - settings.margin * 2.0) * (settings.sheetHeight - settings.margin * 2.0));
    const double usedArea = std::max(1.0, used.area());
    const double conservative = partArea / sheetArea;
    const double packingDensity = partArea / usedArea;
    return std::max(0.0, std::min(1.0, std::max(conservative, packingDensity * conservative)));
}

ValidationSummary validateLayout(
    const Document& document,
    const std::vector<Pose>& poses,
    const EngineSettings& settings,
    ParallelCollisionEvaluator& evaluator,
    WorkerPool& workerPool,
    size_t workerThreadCount) {
    ValidationSummary summary;
    summary.collisionReport = evaluateLayout(document, poses, settings, evaluator, workerPool, workerThreadCount);

    const ClearanceSettings clearance{
        settings.partSpacing,
        settings.margin,
        settings.collisionTolerance
    };

    BroadPhase broad;
    const auto spacingPairs = broad.findCandidatePairs(document.parts, poses, settings.partSpacing);
    for (const auto& [a, b] : spacingPairs) {
        if (a >= document.parts.size() || b >= document.parts.size() || a >= poses.size() || b >= poses.size()) {
            continue;
        }
        if (!partsCollide(document.parts[a], poses[a], document.parts[b], poses[b], settings.collisionTolerance) &&
            !partsRespectSpacing(document.parts[a], poses[a], document.parts[b], poses[b], clearance)) {
            ++summary.spacingFailureCount;
        }
    }

    for (size_t i = 0; i < document.parts.size() && i < poses.size(); ++i) {
        bool invalid = false;
        if (!isPartInsideSheet(document.parts[i], poses[i], document.sheet, settings.collisionTolerance) ||
            overlapsSheetHolesOrForbiddenZones(document.parts[i], poses[i], document.sheet, settings.collisionTolerance)) {
            invalid = true;
        }
        if (!partRespectsSheetMargin(document.parts[i], poses[i], document.sheet, clearance)) {
            invalid = true;
            ++summary.sheetMarginFailureCount;
        }
        if (invalid) {
            ++summary.invalidPartCount;
        }
    }

    return summary;
}

} // namespace

NestingEngine::NestingEngine() = default;

NestingEngine::~NestingEngine() {
    stop();
}

void NestingEngine::setDocument(Document* doc) {
    if (isRunning()) {
        return;
    }
    document_ = doc;
}

void NestingEngine::setSettings(const EngineSettings& settings) {
    if (isRunning()) {
        return;
    }
    settings_ = settings;
}

void NestingEngine::start() {
    if (running_.load()) {
        return;
    }
    if (worker_.joinable()) {
        worker_.join();
    }

    stopRequested_.store(false);
    running_.store(true);
    worker_ = std::thread([this]() { run(); });
}

void NestingEngine::requestStop() {
    stopRequested_.store(true);
}

void NestingEngine::stop() {
    stopRequested_.store(true);
    if (worker_.joinable()) {
        worker_.join();
    }
    running_.store(false);
}

bool NestingEngine::isRunning() const {
    return running_.load();
}

SolverSnapshot NestingEngine::getLatestSnapshot() const {
    std::shared_ptr<const SolverSnapshot> snapshot;
    {
        std::lock_guard<std::mutex> lock(snapshotMutex_);
        snapshot = latestSnapshot_;
    }
    return snapshot ? *snapshot : SolverSnapshot{};
}

SolverResult NestingEngine::getBestResult() const {
    std::lock_guard<std::mutex> lock(resultMutex_);
    return bestResult_;
}

void NestingEngine::publishSnapshot(SolverPhase phase, double progress, const std::vector<Pose>& current, const std::vector<Pose>& best, size_t collisions, double overlap, double utilization, bool running, double elapsedSeconds, size_t validationFailures, size_t invalidParts) {
    auto snapshot = std::make_shared<SolverSnapshot>();
    snapshot->currentPoses = current;
    snapshot->bestPoses = best;
    snapshot->phase = phase;
    snapshot->progress = std::max(0.0, std::min(1.0, progress));
    snapshot->collisionCount = collisions;
    snapshot->overlapScore = overlap;
    snapshot->utilization = utilization;
    snapshot->elapsedSeconds = elapsedSeconds;
    snapshot->validationFailureCount = validationFailures;
    snapshot->invalidPartCount = invalidParts;
    snapshot->running = running;

    std::lock_guard<std::mutex> lock(snapshotMutex_);
    latestSnapshot_ = std::move(snapshot);
}

void NestingEngine::run() {
    const auto started = Clock::now();
    Document* doc = document_;
    EngineSettings settings = settings_;
    std::vector<Pose> poses;
    std::vector<Pose> bestPoses;
    PenaltySystem penalties;

    if (doc == nullptr || doc->parts.empty()) {
        publishSnapshot(SolverPhase::Done, 1.0, poses, bestPoses, 0, 0.0, 0.0, false, elapsedSecondsSince(started));
        running_.store(false);
        return;
    }

    const size_t workerThreadCount = ParallelCollisionEvaluator::resolveThreadCount(settings);
    WorkerPool workerPool(workerThreadCount);
    ParallelCollisionEvaluator collisionEvaluator;
    const auto snapshotInterval = std::chrono::milliseconds(std::max(16, settings.livePreviewIntervalMs));
    auto lastSnapshotTime = Clock::now() - snapshotInterval;

    auto evaluateCurrent = [&]() {
        return evaluateLayout(*doc, poses, settings, collisionEvaluator, workerPool, workerThreadCount);
    };

    auto publishIfDue = [&](SolverPhase phase, double progress, const CollisionReport& report, bool force) {
        const auto now = Clock::now();
        if (!force && now - lastSnapshotTime < snapshotInterval) {
            return;
        }
        lastSnapshotTime = now;
        publishSnapshot(phase, progress, poses, bestPoses, report.collisionCount, report.overlapScore, layoutUtilization(*doc, poses, settings), true, elapsedSecondsSince(started));
    };

    auto finishStopped = [&]() {
        const auto report = evaluateCurrent();
        publishSnapshot(SolverPhase::Stopped, 1.0, poses, bestPoses, report.collisionCount, report.overlapScore, layoutUtilization(*doc, poses, settings), false, elapsedSecondsSince(started));
        running_.store(false);
    };

    const size_t n = doc->parts.size();
    poses.resize(n);
    bestPoses.resize(n);
    for (size_t i = 0; i < n; ++i) {
        poses[i] = doc->parts[i].pose;
        bestPoses[i] = poses[i];
    }

    publishIfDue(SolverPhase::PrepareGeometry, 0.02, CollisionReport{}, true);
    if (stopRequested_.load()) {
        finishStopped();
        return;
    }

    std::vector<size_t> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return scoreArea(doc->parts[a]) > scoreArea(doc->parts[b]);
    });

    PoseSampler sampler;
    const auto rotations = sampler.coarseRotationSamples(settings);
    const auto mirrors = sampler.mirrorSamples(settings);

    const LinearPlacementPlan placementPlan = planForStrategy(settings.placementStrategy);
    const double leftLimit = settings.margin;
    const double rightLimit = settings.sheetWidth - settings.margin;
    const double lowLimit = settings.margin;
    const double highLimit = settings.sheetHeight - settings.margin;
    double x = placementPlan.xDirection > 0 ? leftLimit : rightLimit;
    double y = placementPlan.yDirection > 0 ? lowLimit : highLimit;
    double rowHeight = 0.0;
    double columnWidth = 0.0;

    auto placeLinear = [&](const Part& part) {
        const double remainingPrimary = placementPlan.primaryX
            ? (placementPlan.xDirection > 0 ? rightLimit - x : x - leftLimit)
            : (placementPlan.yDirection > 0 ? highLimit - y : y - lowLimit);
        PlacementChoice choice = choosePlacementChoice(part, rotations, mirrors, std::max(0.0, remainingPrimary), placementPlan.primaryX);
        double width = std::max(1.0, choice.bounds.width());
        double height = std::max(1.0, choice.bounds.height());

        if (placementPlan.primaryX) {
            const bool wraps = placementPlan.xDirection > 0
                ? (x + width > rightLimit && x > leftLimit)
                : (x - width < leftLimit && x < rightLimit);
            if (wraps) {
                x = placementPlan.xDirection > 0 ? leftLimit : rightLimit;
                y += static_cast<double>(placementPlan.yDirection) * (rowHeight + settings.partSpacing);
                rowHeight = 0.0;
                const double rowRemaining = placementPlan.xDirection > 0 ? rightLimit - x : x - leftLimit;
                choice = choosePlacementChoice(part, rotations, mirrors, std::max(0.0, rowRemaining), true);
                width = std::max(1.0, choice.bounds.width());
                height = std::max(1.0, choice.bounds.height());
            }

            const Pose pose = poseFromCornerAnchor(choice, x, y, placementPlan.xDirection, placementPlan.yDirection);
            x += static_cast<double>(placementPlan.xDirection) * (width + settings.partSpacing);
            rowHeight = std::max(rowHeight, height);
            return pose;
        }

        const bool wraps = placementPlan.yDirection > 0
            ? (y + height > highLimit && y > lowLimit)
            : (y - height < lowLimit && y < highLimit);
        if (wraps) {
            y = placementPlan.yDirection > 0 ? lowLimit : highLimit;
            x += static_cast<double>(placementPlan.xDirection) * (columnWidth + settings.partSpacing);
            columnWidth = 0.0;
            const double columnRemaining = placementPlan.yDirection > 0 ? highLimit - y : y - lowLimit;
            choice = choosePlacementChoice(part, rotations, mirrors, std::max(0.0, columnRemaining), false);
            width = std::max(1.0, choice.bounds.width());
            height = std::max(1.0, choice.bounds.height());
        }

        const Pose pose = poseFromCornerAnchor(choice, x, y, placementPlan.xDirection, placementPlan.yDirection);
        y += static_cast<double>(placementPlan.yDirection) * (height + settings.partSpacing);
        columnWidth = std::max(columnWidth, width);
        return pose;
    };

    auto placeAnchored = [&](const Part& part, size_t placed) {
        const double sheetSpan = std::max(1.0, placementPlan.primaryX ? rightLimit - leftLimit : highLimit - lowLimit);
        const PlacementChoice choice = choosePlacementChoice(part, rotations, mirrors, sheetSpan, true);
        const double width = std::max(1.0, choice.bounds.width());
        const double height = std::max(1.0, choice.bounds.height());
        const double partSpan = std::max(width, height);

        Vec2 anchor;
        if (settings.placementStrategy == PlacementStrategy::OutsideIn) {
            anchor = outsideInAnchor(placed, partSpan, settings);
        } else if (settings.placementStrategy == PlacementStrategy::UserPoints && !doc->sheet.getUserPlacementPoints().empty()) {
            const auto& anchors = doc->sheet.getUserPlacementPoints();
            anchor = anchors[placed % anchors.size()];
            const size_t layer = placed / anchors.size();
            anchor.x += static_cast<double>(layer) * (width + settings.partSpacing);
        } else {
            anchor = centerOutAnchor(placed, partSpan, settings);
        }

        anchor.x = clampToSheet(anchor.x, leftLimit + width * 0.5, rightLimit - width * 0.5);
        anchor.y = clampToSheet(anchor.y, lowLimit + height * 0.5, highLimit - height * 0.5);
        return poseFromCenterAnchor(choice, anchor);
    };

    for (size_t placed = 0; placed < order.size(); ++placed) {
        if (stopRequested_.load()) {
            finishStopped();
            return;
        }
        const size_t idx = order[placed];
        const Part& part = doc->parts[idx];

        const bool useAnchorPlacement = usesAnchorPlacement(settings.placementStrategy) &&
            (settings.placementStrategy != PlacementStrategy::UserPoints || !doc->sheet.getUserPlacementPoints().empty());
        poses[idx] = useAnchorPlacement ? placeAnchored(part, placed) : placeLinear(part);
        bestPoses = poses;

        const auto report = evaluateCurrent();
        const double progress = 0.05 + 0.35 * (static_cast<double>(placed + 1) / static_cast<double>(order.size()));
        publishIfDue(SolverPhase::InitialPlacement, progress, report, placed + 1 == order.size());
    }

    publishIfDue(SolverPhase::Exploration, 0.45, evaluateCurrent(), true);
    if (stopRequested_.load()) {
        finishStopped();
        return;
    }

    LocalSearch localSearch;
    for (int i = 0; i < 4; ++i) {
        localSearch.resolveSimpleCollisions(*doc, settings, poses);
        const auto report = evaluateCurrent();
        for (const auto& pair : report.pairs) {
            penalties.observeCollision(pair.a, pair.b);
        }
        publishIfDue(SolverPhase::CollisionResolution, 0.50 + 0.10 * static_cast<double>(i + 1), report, report.collisionCount == 0 || i == 3);
        if (report.collisionCount == 0 || stopRequested_.load()) {
            break;
        }
    }
    if (stopRequested_.load()) {
        finishStopped();
        return;
    }

    Compression compression;
    compression.compressByStrategy(*doc, settings, poses);
    auto report = evaluateCurrent();
    bestPoses = poses;
    publishIfDue(SolverPhase::Compression, 0.76, report, true);
    if (stopRequested_.load()) {
        finishStopped();
        return;
    }

    // 0.001 degree precision belongs to this local refinement stage, not to brute-force angle scanning.
    publishIfDue(SolverPhase::UltraRefinement, 0.90, report, true);
    if (stopRequested_.load()) {
        finishStopped();
        return;
    }

    const ValidationSummary validation = validateLayout(*doc, poses, settings, collisionEvaluator, workerPool, workerThreadCount);
    publishSnapshot(
        SolverPhase::FinalValidation,
        0.97,
        poses,
        bestPoses,
        validation.collisionReport.collisionCount,
        validation.collisionReport.overlapScore,
        layoutUtilization(*doc, poses, settings),
        true,
        elapsedSecondsSince(started),
        validation.validationFailureCount(),
        validation.invalidPartCount);

    const double utilization = layoutUtilization(*doc, poses, settings);
    {
        std::lock_guard<std::mutex> lock(resultMutex_);
        bestResult_.bestPoses = bestPoses;
        bestResult_.collisionCount = validation.collisionReport.collisionCount;
        bestResult_.overlapScore = validation.collisionReport.overlapScore;
        bestResult_.utilization = utilization;
        bestResult_.validationFailureCount = validation.validationFailureCount();
        bestResult_.invalidPartCount = validation.invalidPartCount;
        bestResult_.valid = validation.validationFailureCount() == 0;
    }

    publishSnapshot(
        SolverPhase::Done,
        1.0,
        poses,
        bestPoses,
        validation.collisionReport.collisionCount,
        validation.collisionReport.overlapScore,
        utilization,
        false,
        elapsedSecondsSince(started),
        validation.validationFailureCount(),
        validation.invalidPartCount);
    running_.store(false);
}

} // namespace nest
