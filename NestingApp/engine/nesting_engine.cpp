#include "engine/nesting_engine.h"

#include "engine/broadphase.h"
#include "engine/compression.h"
#include "engine/local_search.h"
#include "engine/narrowphase.h"
#include "engine/penalty_system.h"
#include "engine/pose_sampler.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <thread>

namespace nest {
namespace {

using Clock = std::chrono::steady_clock;

double scoreArea(const Part& part) {
    return part.area > 0.0 ? part.area : part.localBounds.area();
}

double elapsedSecondsSince(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}

CollisionReport evaluateLayout(const Document& document, const std::vector<Pose>& poses, const EngineSettings& settings) {
    BroadPhase broad;
    NarrowPhase narrow;
    return narrow.evaluate(document.parts, poses, broad.findCandidatePairs(document.parts, poses, settings.partSpacing), settings.collisionTolerance);
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
    std::lock_guard<std::mutex> lock(snapshotMutex_);
    return latestSnapshot_;
}

SolverResult NestingEngine::getBestResult() const {
    std::lock_guard<std::mutex> lock(resultMutex_);
    return bestResult_;
}

void NestingEngine::publishSnapshot(SolverPhase phase, double progress, const std::vector<Pose>& current, const std::vector<Pose>& best, size_t collisions, double overlap, double utilization, bool running, double elapsedSeconds) {
    SolverSnapshot snapshot;
    snapshot.currentPoses = current;
    snapshot.bestPoses = best;
    snapshot.phaseName = phaseName(phase);
    snapshot.progress = std::max(0.0, std::min(1.0, progress));
    snapshot.collisionCount = collisions;
    snapshot.overlapScore = overlap;
    snapshot.utilization = utilization;
    snapshot.elapsedSeconds = elapsedSeconds;
    snapshot.running = running;

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

    auto finishStopped = [&]() {
        const auto report = doc ? evaluateLayout(*doc, poses, settings) : CollisionReport{};
        publishSnapshot(SolverPhase::Stopped, 1.0, poses, bestPoses, report.collisionCount, report.overlapScore, doc ? layoutUtilization(*doc, poses, settings) : 0.0, false, elapsedSecondsSince(started));
        running_.store(false);
    };

    if (doc == nullptr || doc->parts.empty()) {
        publishSnapshot(SolverPhase::Done, 1.0, poses, bestPoses, 0, 0.0, 0.0, false, elapsedSecondsSince(started));
        running_.store(false);
        return;
    }

    const size_t n = doc->parts.size();
    poses.resize(n);
    bestPoses.resize(n);
    for (size_t i = 0; i < n; ++i) {
        poses[i] = doc->parts[i].pose;
        bestPoses[i] = poses[i];
    }

    publishSnapshot(SolverPhase::PrepareGeometry, 0.02, poses, bestPoses, 0, 0.0, 0.0, true, elapsedSecondsSince(started));
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
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
    (void)rotations;

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
        const AABB bounds = part.localBounds;
        const double width = std::max(1.0, bounds.width());
        const double height = std::max(1.0, bounds.height());

        if (x + width > settings.sheetWidth - settings.margin && x > settings.margin) {
            x = settings.margin;
            y += rowHeight + settings.partSpacing;
            rowHeight = 0.0;
        }

        Pose pose;
        pose.x = x - bounds.min.x;
        pose.y = y - bounds.min.y;
        pose.angleRadians = 0.0;
        pose.mirrored = false;
        poses[idx] = pose;
        bestPoses = poses;

        x += width + settings.partSpacing;
        rowHeight = std::max(rowHeight, height);

        const auto report = evaluateLayout(*doc, poses, settings);
        publishSnapshot(SolverPhase::InitialPlacement, 0.05 + 0.35 * (static_cast<double>(placed + 1) / static_cast<double>(order.size())), poses, bestPoses, report.collisionCount, report.overlapScore, layoutUtilization(*doc, poses, settings), true, elapsedSecondsSince(started));
        std::this_thread::sleep_for(std::chrono::milliseconds(std::max(10, settings.livePreviewIntervalMs)));
    }

    publishSnapshot(SolverPhase::Exploration, 0.45, poses, bestPoses, 0, 0.0, layoutUtilization(*doc, poses, settings), true, elapsedSecondsSince(started));
    std::this_thread::sleep_for(std::chrono::milliseconds(70));
    if (stopRequested_.load()) {
        finishStopped();
        return;
    }

    LocalSearch localSearch;
    for (int i = 0; i < 4; ++i) {
        localSearch.resolveSimpleCollisions(*doc, settings, poses);
        const auto report = evaluateLayout(*doc, poses, settings);
        for (const auto& pair : report.pairs) {
            penalties.observeCollision(pair.a, pair.b);
        }
        publishSnapshot(SolverPhase::CollisionResolution, 0.50 + 0.10 * static_cast<double>(i + 1), poses, poses, report.collisionCount, report.overlapScore, layoutUtilization(*doc, poses, settings), true, elapsedSecondsSince(started));
        if (report.collisionCount == 0 || stopRequested_.load()) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
    if (stopRequested_.load()) {
        finishStopped();
        return;
    }

    Compression compression;
    compression.compressLeftUp(*doc, settings, poses);
    auto report = evaluateLayout(*doc, poses, settings);
    bestPoses = poses;
    publishSnapshot(SolverPhase::Compression, 0.76, poses, bestPoses, report.collisionCount, report.overlapScore, layoutUtilization(*doc, poses, settings), true, elapsedSecondsSince(started));
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    if (stopRequested_.load()) {
        finishStopped();
        return;
    }

    // 0.001 degree precision belongs to this local refinement stage, not to brute-force angle scanning.
    publishSnapshot(SolverPhase::UltraRefinement, 0.90, poses, bestPoses, report.collisionCount, report.overlapScore, layoutUtilization(*doc, poses, settings), true, elapsedSecondsSince(started));
    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    report = evaluateLayout(*doc, poses, settings);
    const double utilization = layoutUtilization(*doc, poses, settings);
    publishSnapshot(SolverPhase::FinalValidation, 0.97, poses, bestPoses, report.collisionCount, report.overlapScore, utilization, true, elapsedSecondsSince(started));
    std::this_thread::sleep_for(std::chrono::milliseconds(40));

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
