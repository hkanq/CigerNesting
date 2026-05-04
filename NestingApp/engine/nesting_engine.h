#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/solver_state.h"
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

namespace nest {

class NestingEngine {
public:
    NestingEngine();
    ~NestingEngine();

    NestingEngine(const NestingEngine&) = delete;
    NestingEngine& operator=(const NestingEngine&) = delete;

    void setDocument(Document* doc);
    void setSettings(const EngineSettings& settings);

    void start();
    void requestStop();
    void stop();
    bool isRunning() const;

    SolverSnapshot getLatestSnapshot() const;
    SolverResult getBestResult() const;

private:
    void run();
    void publishSnapshot(SolverPhase phase, SolverStrategy strategy, double progress, const std::vector<Pose>& current, const std::vector<Pose>& best, size_t collisions, double overlap, double utilization, bool running, double elapsedSeconds, size_t validationFailures = 0, size_t invalidParts = 0, const SolverStats& stats = {}, const ActiveMoveSummary& activeMoves = {}, uint64_t versionId = 0, bool layoutChanged = false, size_t lastMovedPart = kNoPartIndex, SolverStrategy lastMoveStrategy = SolverStrategy::Idle, bool bestUpdated = false);

    Document* document_ = nullptr;
    EngineSettings settings_{};

    std::thread worker_;
    std::atomic_bool running_{false};
    std::atomic_bool stopRequested_{false};

    mutable std::mutex snapshotMutex_;
    std::shared_ptr<const SolverSnapshot> latestSnapshot_;

    mutable std::mutex resultMutex_;
    SolverResult bestResult_;
};

} // namespace nest
