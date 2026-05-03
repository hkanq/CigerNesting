#include "engine/nesting_engine.h"

#include "engine/broadphase.h"
#include "engine/compression.h"
#include "engine/local_search.h"
#include "engine/parallel_collision_evaluator.h"
#include "engine/penalty_system.h"
#include "engine/pose_sampler.h"
#include "engine/worker_pool.h"
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
    const EngineSettings& settings,
    double rowX) {
    PlacementChoice best;
    double bestScore = std::numeric_limits<double>::max();
    const double rightLimit = settings.sheetWidth - settings.margin;

    for (const double angle : rotationSamples) {
        for (const bool mirrored : mirrorSamples) {
            const AABB bounds = orientedLocalBounds(part, angle, mirrored);
            if (!bounds.isValid()) {
                continue;
            }
            const double width = std::max(1.0, bounds.width());
            const double height = std::max(1.0, bounds.height());
            const bool fitsCurrentRow = rowX + width <= rightLimit + 1e-6;
            const double fitPenalty = fitsCurrentRow ? 0.0 : 1000000.0;
            const double score = fitPenalty + width + height * 0.05;
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

void NestingEngine::publishSnapshot(SolverPhase phase, double progress, const std::vector<Pose>& current, const std::vector<Pose>& best, size_t collisions, double overlap, double utilization, bool running, double elapsedSeconds) {
    auto snapshot = std::make_shared<SolverSnapshot>();
    snapshot->currentPoses = current;
    snapshot->bestPoses = best;
    snapshot->phase = phase;
    snapshot->progress = std::max(0.0, std::min(1.0, progress));
    snapshot->collisionCount = collisions;
    snapshot->overlapScore = overlap;
    snapshot->utilization = utilization;
    snapshot->elapsedSeconds = elapsedSeconds;
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

    double x = settings.margin;
    double y = settings.margin;
    double rowHeight = 0.0;

    for (size_t placed = 0; placed < order.size(); ++placed) {
        if (stopRequested_.load()) {
            finishStopped();
            return;
        }
        const size_t idx = order[placed];
        const Part& part = doc->parts[idx];

        PlacementChoice choice = choosePlacementChoice(part, rotations, mirrors, settings, x);
        double width = std::max(1.0, choice.bounds.width());
        double height = std::max(1.0, choice.bounds.height());

        if (x + width > settings.sheetWidth - settings.margin && x > settings.margin) {
            x = settings.margin;
            y += rowHeight + settings.partSpacing;
            rowHeight = 0.0;
            choice = choosePlacementChoice(part, rotations, mirrors, settings, x);
            width = std::max(1.0, choice.bounds.width());
            height = std::max(1.0, choice.bounds.height());
        }

        Pose pose;
        pose.x = x - choice.bounds.min.x;
        pose.y = y - choice.bounds.min.y;
        pose.angleRadians = choice.angleRadians;
        pose.mirrored = choice.mirrored;
        poses[idx] = pose;
        bestPoses = poses;

        x += width + settings.partSpacing;
        rowHeight = std::max(rowHeight, height);

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
    compression.compressLeftUp(*doc, settings, poses);
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

    report = evaluateCurrent();
    publishIfDue(SolverPhase::FinalValidation, 0.97, report, true);

    const double utilization = layoutUtilization(*doc, poses, settings);
    {
        std::lock_guard<std::mutex> lock(resultMutex_);
        bestResult_.bestPoses = bestPoses;
        bestResult_.collisionCount = report.collisionCount;
        bestResult_.overlapScore = report.overlapScore;
        bestResult_.utilization = utilization;
        bestResult_.valid = report.collisionCount == 0;
    }

    publishSnapshot(SolverPhase::Done, 1.0, poses, bestPoses, report.collisionCount, report.overlapScore, utilization, false, elapsedSecondsSince(started));
    running_.store(false);
}

} // namespace nest
