#include "engine/nesting_engine.h"

#include "engine/layout_score.h"
#include "engine/multi_start_solver.h"
#include "engine/penalty_system.h"
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

void NestingEngine::publishSnapshot(SolverPhase phase, SolverStrategy strategy, double progress, const std::vector<Pose>& current, const std::vector<Pose>& best, size_t collisions, double overlap, double utilization, bool running, double elapsedSeconds, size_t validationFailures, size_t invalidParts, const SolverStats& stats, const ActiveMoveSummary& activeMoves, uint64_t versionId, bool layoutChanged, size_t lastMovedPart, SolverStrategy lastMoveStrategy, bool bestUpdated, const std::vector<size_t>& changedParts, size_t rebuildAttempt, size_t beamDepth, size_t subsetSize, bool previewTemporary) {
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
    snapshot->stats = stats;
    snapshot->currentStrategy = strategy;
    snapshot->activeMoves = activeMoves;
    snapshot->versionId = versionId;
    snapshot->layoutChanged = layoutChanged;
    snapshot->lastMovedPart = lastMovedPart;
    snapshot->lastMoveStrategy = lastMoveStrategy;
    snapshot->bestUpdated = bestUpdated;
    snapshot->changedParts = changedParts;
    snapshot->rebuildAttempt = rebuildAttempt;
    snapshot->beamDepth = beamDepth;
    snapshot->subsetSize = subsetSize;
    snapshot->previewTemporary = previewTemporary;
    snapshot->bestValidUtilization = utilization;
    snapshot->currentTrajectoryUtilization = utilization;
    snapshot->bestUpdateCount = stats.bestUpdates;
    snapshot->destroyBestUpdateCount = stats.destroyBestUpdates;
    snapshot->running = running;

    std::lock_guard<std::mutex> lock(snapshotMutex_);
    latestSnapshot_ = std::move(snapshot);
}

void NestingEngine::run() {
    const auto started = Clock::now();
    Document* doc = document_;
    EngineSettings settings = settings_;

    if (doc == nullptr || doc->parts.empty()) {
        publishSnapshot(SolverPhase::Done, SolverStrategy::Done, 1.0, {}, {}, 0, 0.0, 0.0, false, elapsedSecondsSince(started));
        running_.store(false);
        return;
    }

    LayoutState latestCurrent;
    LayoutState latestBest;
    SolverStats latestStats;
    uint64_t latestVersionId = 0;
    auto publishState = [&](const SolverProgress& progress, bool force, bool solverRunning) {
        if (!force && !progress.layoutChanged && progress.versionId == latestVersionId) {
            return;
        }
        if (!force && progress.versionId != 0 && progress.versionId == latestVersionId) {
            return;
        }
        if (progress.versionId != 0) {
            latestVersionId = progress.versionId;
        }
        latestCurrent = progress.current;
        latestBest = progress.best;
        latestStats = progress.stats;
        const LayoutState& display = progress.layoutChanged && !progress.current.poses.empty() && (progress.current.valid() || progress.previewTemporary)
            ? progress.current
            : (progress.best.valid() && !progress.best.poses.empty() ? progress.best : progress.current);
        publishSnapshot(
            progress.phase,
            progress.currentStrategy,
            progress.progress,
            display.poses,
            progress.best.poses,
            static_cast<size_t>(std::max(0, display.collisionCount)),
            display.overlapPenalty,
            display.utilization,
            solverRunning,
            elapsedSecondsSince(started),
            static_cast<size_t>(std::max(0, display.collisionCount + display.invalidPartCount)) +
                static_cast<size_t>(display.spacingPenalty > 0.0 ? 1 : 0) +
                static_cast<size_t>(display.sheetPenalty > 0.0 ? 1 : 0),
            static_cast<size_t>(std::max(0, display.invalidPartCount)),
            progress.stats,
            progress.activeMoves,
            latestVersionId,
            progress.layoutChanged,
            progress.lastMovedPart,
            progress.lastMoveStrategy,
            progress.bestUpdated,
            progress.changedParts,
            progress.rebuildAttempt,
            progress.beamDepth,
            progress.subsetSize,
            progress.previewTemporary);
    };

    publishSnapshot(SolverPhase::PrepareGeometry, SolverStrategy::AdaptiveSearch, 0.02, {}, {}, 0, 0.0, 0.0, true, elapsedSecondsSince(started));
    if (stopRequested_.load()) {
        publishSnapshot(SolverPhase::Stopped, SolverStrategy::Idle, 1.0, {}, {}, 0, 0.0, 0.0, false, elapsedSecondsSince(started));
        running_.store(false);
        return;
    }

    MultiStartSolver solver;
    const LayoutState best = solver.solve(*doc, settings, stopRequested_, [&](const SolverProgress& progress) {
        publishState(progress, false, true);
    });
    latestStats = solver.lastStats();

    LayoutScore scorer;
    PenaltySystem finalPenalties;
    LayoutState finalBest = best.poses.empty() ? latestBest : scorer.evaluate(*doc, settings, best.poses, &finalPenalties);
    if (!finalBest.poses.empty()) {
        finalBest = scorer.evaluate(*doc, settings, finalBest.poses, &finalPenalties);
    }
    LayoutState finalCurrent = finalBest.valid() ? finalBest : (latestCurrent.poses.empty() ? finalBest : latestCurrent);
    const SolverPhase finalPhase = stopRequested_.load() ? SolverPhase::Stopped : SolverPhase::FinalValidation;
    SolverProgress finalValidationProgress;
    finalValidationProgress.phase = finalPhase;
    finalValidationProgress.currentStrategy = SolverStrategy::Done;
    finalValidationProgress.progress = stopRequested_.load() ? 1.0 : 0.97;
    finalValidationProgress.current = finalCurrent;
    finalValidationProgress.best = finalBest;
    finalValidationProgress.elapsedSeconds = elapsedSecondsSince(started);
    finalValidationProgress.stats = latestStats;
    finalValidationProgress.versionId = latestVersionId;
    publishState(finalValidationProgress, true, !stopRequested_.load());

    const double utilization = finalBest.utilization;
    {
        std::lock_guard<std::mutex> lock(resultMutex_);
        bestResult_.bestPoses = finalBest.poses;
        bestResult_.collisionCount = static_cast<size_t>(std::max(0, finalBest.collisionCount));
        bestResult_.overlapScore = finalBest.overlapPenalty;
        bestResult_.utilization = utilization;
        bestResult_.validationFailureCount = static_cast<size_t>(std::max(0, finalBest.collisionCount + finalBest.invalidPartCount)) +
            static_cast<size_t>(finalBest.spacingPenalty > 0.0 ? 1 : 0) +
            static_cast<size_t>(finalBest.sheetPenalty > 0.0 ? 1 : 0);
        bestResult_.invalidPartCount = static_cast<size_t>(std::max(0, finalBest.invalidPartCount));
        bestResult_.stats = latestStats;
        bestResult_.currentStrategy = finalBest.valid() ? SolverStrategy::Done : SolverStrategy::Idle;
        bestResult_.versionId = latestVersionId;
        bestResult_.valid = finalBest.valid();
    }

    const bool finalValid = finalBest.valid();
    const SolverPhase terminalPhase = stopRequested_.load()
        ? SolverPhase::Stopped
        : (finalValid ? SolverPhase::Done : SolverPhase::NoValidLayout);
    const LayoutState& terminalDisplay = finalValid ? finalBest : finalCurrent;

    publishSnapshot(
        terminalPhase,
        finalValid ? SolverStrategy::Done : SolverStrategy::Idle,
        1.0,
        terminalDisplay.poses,
        finalBest.poses,
        static_cast<size_t>(std::max(0, terminalDisplay.collisionCount)),
        terminalDisplay.overlapPenalty,
        terminalDisplay.utilization,
        false,
        elapsedSecondsSince(started),
        static_cast<size_t>(std::max(0, terminalDisplay.collisionCount + terminalDisplay.invalidPartCount)) +
        static_cast<size_t>(terminalDisplay.spacingPenalty > 0.0 ? 1 : 0) +
            static_cast<size_t>(terminalDisplay.sheetPenalty > 0.0 ? 1 : 0),
        static_cast<size_t>(std::max(0, terminalDisplay.invalidPartCount)),
        latestStats,
        {},
        latestVersionId,
        false,
        kNoPartIndex,
        finalValid ? SolverStrategy::Done : SolverStrategy::Idle,
        false);
    running_.store(false);
}

} // namespace nest
