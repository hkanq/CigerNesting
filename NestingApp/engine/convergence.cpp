#include "engine/convergence.h"

#include <algorithm>
#include <cmath>

namespace nest {
namespace {

double elapsedMs(std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
}

} // namespace

ConvergenceTracker::ConvergenceTracker(ConvergenceCriteria criteria)
    : criteria_(criteria) {}

void ConvergenceTracker::reset(const LayoutState& initialBest, const SolverStats& stats, std::chrono::steady_clock::time_point now) {
    bestScore_ = initialBest.totalScore;
    bestUtilization_ = initialBest.utilization;
    lastAcceptedMoves_ = stats.acceptedMoves;
    lastImprovement_ = now;
    totalPasses_ = 0;
    stagnantPasses_ = 0;
    initialized_ = true;
}

void ConvergenceTracker::observeAttempt(const LayoutState& best, const SolverStats& stats, std::chrono::steady_clock::time_point now) {
    if (!initialized_) {
        reset(best, stats, now);
        return;
    }

    ++totalPasses_;
    const double scoreScale = std::max(1.0, std::abs(bestScore_));
    const bool scoreImproved = best.valid() && best.totalScore < bestScore_ - criteria_.minImprovementEpsilon * scoreScale;
    const bool utilizationImproved = best.valid() && best.utilization > bestUtilization_ + criteria_.utilizationPlateauEpsilon;
    const size_t acceptedDelta = stats.acceptedMoves >= lastAcceptedMoves_ ? stats.acceptedMoves - lastAcceptedMoves_ : 0;
    lastAcceptedMoves_ = stats.acceptedMoves;

    if (scoreImproved || utilizationImproved) {
        bestScore_ = best.totalScore;
        bestUtilization_ = best.utilization;
        lastImprovement_ = now;
        stagnantPasses_ = 0;
        return;
    }

    if (acceptedDelta <= criteria_.acceptedMovesLowThreshold) {
        ++stagnantPasses_;
    }
}

bool ConvergenceTracker::shouldStop(std::chrono::steady_clock::time_point now) const {
    if (!initialized_) {
        return false;
    }
    if (criteria_.maxTotalPasses > 0 && totalPasses_ >= criteria_.maxTotalPasses) {
        return true;
    }
    const bool staleByPasses = stagnantPasses_ >= criteria_.noImprovementPasses;
    const bool staleByTime = elapsedMs(lastImprovement_, now) >= criteria_.noImprovementMs;
    return criteria_.allPhasesStalled ? (staleByPasses && staleByTime) : (staleByPasses || staleByTime);
}

ConvergenceCriteria criteriaForSettings(const EngineSettings& settings, size_t partCount) {
    ConvergenceCriteria criteria;
    const bool autoMode = autoTimeLimitEnabled(settings);
    const int partFactor = static_cast<int>(std::max<size_t>(1, partCount / 100));
    switch (settings.performanceProfile) {
    case PerformanceProfile::Fast:
        criteria.noImprovementPasses = autoMode ? 3 : 4;
        criteria.noImprovementMs = autoMode ? 900.0 : 1400.0;
        criteria.maxTotalPasses = std::max(4, 5 + partFactor);
        criteria.acceptedMovesLowThreshold = 1;
        criteria.autoSafetyCapSeconds = std::min(12.0, 4.0 + static_cast<double>(partCount) * 0.010);
        break;
    case PerformanceProfile::Maximum:
        criteria.noImprovementPasses = autoMode ? 10 : 12;
        criteria.noImprovementMs = autoMode ? 6500.0 : 9000.0;
        criteria.maxTotalPasses = std::max(16, 18 + partFactor * 4);
        criteria.acceptedMovesLowThreshold = 2;
        criteria.autoSafetyCapSeconds = std::min(90.0, 18.0 + static_cast<double>(partCount) * 0.055);
        break;
    case PerformanceProfile::Balanced:
    default:
        criteria.noImprovementPasses = autoMode ? 6 : 8;
        criteria.noImprovementMs = autoMode ? 3000.0 : 5000.0;
        criteria.maxTotalPasses = std::max(10, 12 + partFactor * 2);
        criteria.acceptedMovesLowThreshold = 1;
        criteria.autoSafetyCapSeconds = std::min(45.0, 9.0 + static_cast<double>(partCount) * 0.030);
        break;
    }
    criteria.minImprovementEpsilon = settings.performanceProfile == PerformanceProfile::Maximum ? 5e-6 : 2e-5;
    criteria.utilizationPlateauEpsilon = settings.performanceProfile == PerformanceProfile::Maximum ? 5e-5 : 1e-4;
    criteria.allPhasesStalled = true;
    return criteria;
}

double effectiveSafetyTimeLimitSeconds(const EngineSettings& settings, size_t partCount) {
    if (!autoTimeLimitEnabled(settings)) {
        return std::max(0.25, settings.timeLimitSeconds);
    }
    return criteriaForSettings(settings, partCount).autoSafetyCapSeconds;
}

bool autoTimeLimitEnabled(const EngineSettings& settings) {
    return settings.timeLimitSeconds <= 0.0;
}

} // namespace nest
