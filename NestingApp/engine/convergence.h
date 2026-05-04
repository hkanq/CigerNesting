#pragma once

#include "engine/engine_settings.h"
#include "engine/layout_state.h"
#include "engine/solver_state.h"
#include <chrono>
#include <cstddef>

namespace nest {

struct ConvergenceCriteria {
    int noImprovementPasses = 6;
    int maxTotalPasses = 24;
    double noImprovementMs = 3000.0;
    double minImprovementEpsilon = 1e-4;
    double utilizationPlateauEpsilon = 1e-4;
    size_t acceptedMovesLowThreshold = 1;
    bool allPhasesStalled = true;
    double autoSafetyCapSeconds = 30.0;
};

class ConvergenceTracker {
public:
    explicit ConvergenceTracker(ConvergenceCriteria criteria);

    void reset(const LayoutState& initialBest, const SolverStats& stats, std::chrono::steady_clock::time_point now);
    void observeAttempt(const LayoutState& best, const SolverStats& stats, std::chrono::steady_clock::time_point now);
    bool shouldStop(std::chrono::steady_clock::time_point now) const;
    int totalPasses() const { return totalPasses_; }
    int stagnantPasses() const { return stagnantPasses_; }

private:
    ConvergenceCriteria criteria_;
    double bestScore_ = 0.0;
    double bestUtilization_ = 0.0;
    size_t lastAcceptedMoves_ = 0;
    std::chrono::steady_clock::time_point lastImprovement_{};
    int totalPasses_ = 0;
    int stagnantPasses_ = 0;
    bool initialized_ = false;
};

ConvergenceCriteria criteriaForSettings(const EngineSettings& settings, size_t partCount);
double effectiveSafetyTimeLimitSeconds(const EngineSettings& settings, size_t partCount);
bool autoTimeLimitEnabled(const EngineSettings& settings);

} // namespace nest
