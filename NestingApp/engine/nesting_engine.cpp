#include "engine/nesting_engine.h"

#include "engine/multi_start_solver.h"
#include <algorithm>
#include <chrono>
#include <memory>

namespace nest {
namespace {

using Clock = std::chrono::steady_clock;

double elapsedSecondsSince(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
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

    if (doc == nullptr || doc->parts.empty()) {
        publishSnapshot(SolverPhase::Done, 1.0, {}, {}, 0, 0.0, 0.0, false, elapsedSecondsSince(started));
        running_.store(false);
        return;
    }

    const auto snapshotInterval = std::chrono::milliseconds(std::max(16, settings.livePreviewIntervalMs));
    auto lastSnapshotTime = Clock::now() - snapshotInterval;

    LayoutState latestCurrent;
    LayoutState latestBest;
    auto publishState = [&](SolverPhase phase, double progress, const LayoutState& current, const LayoutState& best, bool force, bool solverRunning) {
        const auto now = Clock::now();
        if (!force && now - lastSnapshotTime < snapshotInterval) {
            return;
        }
        lastSnapshotTime = now;
        latestCurrent = current;
        latestBest = best;
        publishSnapshot(
            phase,
            progress,
            current.poses,
            best.poses,
            static_cast<size_t>(std::max(0, current.collisionCount)),
            current.overlapPenalty,
            current.utilization,
            solverRunning,
            elapsedSecondsSince(started),
            static_cast<size_t>(std::max(0, current.collisionCount + current.invalidPartCount)) + static_cast<size_t>(current.spacingPenalty > 0.0 ? 1 : 0),
            static_cast<size_t>(std::max(0, current.invalidPartCount)));
    };

    publishSnapshot(SolverPhase::PrepareGeometry, 0.02, {}, {}, 0, 0.0, 0.0, true, elapsedSecondsSince(started));
    if (stopRequested_.load()) {
        publishSnapshot(SolverPhase::Stopped, 1.0, {}, {}, 0, 0.0, 0.0, false, elapsedSecondsSince(started));
        running_.store(false);
        return;
    }

    MultiStartSolver solver;
    const LayoutState best = solver.solve(*doc, settings, stopRequested_, [&](const SolverProgress& progress) {
        publishState(progress.phase, progress.progress, progress.current, progress.best, false, true);
    });

    LayoutState finalCurrent = latestCurrent.poses.empty() ? best : latestCurrent;
    const LayoutState finalBest = best.poses.empty() ? latestBest : best;
    const SolverPhase finalPhase = stopRequested_.load() ? SolverPhase::Stopped : SolverPhase::FinalValidation;
    publishState(finalPhase, stopRequested_.load() ? 1.0 : 0.97, finalCurrent, finalBest, true, !stopRequested_.load());

    const double utilization = finalBest.utilization;
    {
        std::lock_guard<std::mutex> lock(resultMutex_);
        bestResult_.bestPoses = finalBest.poses;
        bestResult_.collisionCount = static_cast<size_t>(std::max(0, finalBest.collisionCount));
        bestResult_.overlapScore = finalBest.overlapPenalty;
        bestResult_.utilization = utilization;
        bestResult_.validationFailureCount = static_cast<size_t>(std::max(0, finalBest.collisionCount + finalBest.invalidPartCount)) + static_cast<size_t>(finalBest.spacingPenalty > 0.0 ? 1 : 0);
        bestResult_.invalidPartCount = static_cast<size_t>(std::max(0, finalBest.invalidPartCount));
        bestResult_.valid = finalBest.valid();
    }

    publishSnapshot(
        stopRequested_.load() ? SolverPhase::Stopped : SolverPhase::Done,
        1.0,
        finalCurrent.poses,
        finalBest.poses,
        static_cast<size_t>(std::max(0, finalBest.collisionCount)),
        finalBest.overlapPenalty,
        utilization,
        false,
        elapsedSecondsSince(started),
        static_cast<size_t>(std::max(0, finalBest.collisionCount + finalBest.invalidPartCount)) + static_cast<size_t>(finalBest.spacingPenalty > 0.0 ? 1 : 0),
        static_cast<size_t>(std::max(0, finalBest.invalidPartCount)));
    running_.store(false);
}

} // namespace nest
