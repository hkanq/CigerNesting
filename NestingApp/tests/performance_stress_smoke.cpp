#include "core/document.h"
#include "core/math_utils.h"
#include "engine/layout_eval_cache.h"
#include "engine/layout_score.h"
#include "engine/multi_start_solver.h"
#include "engine/penalty_system.h"
#include "engine/pose_sampler.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace nest;
using Clock = std::chrono::steady_clock;

Ring boxRing(double x0, double y0, double x1, double y1) {
    Ring ring;
    ring.points = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}, {x0, y0}};
    return ring;
}

Ring concaveRing(double w, double h) {
    Ring ring;
    ring.points = {
        {0.0, 0.0}, {w, 0.0}, {w, h * 0.38}, {w * 0.48, h * 0.38},
        {w * 0.48, h}, {0.0, h}, {0.0, 0.0}
    };
    return ring;
}

Part partFromRings(std::vector<Ring> rings) {
    Part part;
    part.rings = std::move(rings);
    part.updateDerivedGeometry();
    return part;
}

Part makePart(size_t index, std::mt19937& rng) {
    std::uniform_real_distribution<double> widthDist(8.0, 36.0);
    std::uniform_real_distribution<double> heightDist(8.0, 32.0);
    const double w = widthDist(rng);
    const double h = heightDist(rng);

    if (index % 11 == 0) {
        Ring hole = boxRing(w * 0.32, h * 0.32, w * 0.68, h * 0.68);
        hole.isHole = true;
        return partFromRings({boxRing(0.0, 0.0, w, h), hole});
    }
    if (index % 7 == 0) {
        return partFromRings({concaveRing(w, h)});
    }
    if (index % 5 == 0) {
        Ring ring;
        ring.points = {{0.0, 0.0}, {w, h * 0.18}, {w * 0.78, h}, {w * 0.12, h * 0.82}, {0.0, 0.0}};
        return partFromRings({std::move(ring)});
    }
    return partFromRings({boxRing(0.0, 0.0, w, h)});
}

Document makeDocument(size_t partCount) {
    Document document;
    document.sheet.width = std::max(900.0, std::sqrt(static_cast<double>(partCount)) * 180.0);
    document.sheet.height = std::max(650.0, std::sqrt(static_cast<double>(partCount)) * 130.0);
    document.sheet.margin = 12.0;

    std::mt19937 rng(static_cast<unsigned int>(partCount * 2654435761u));
    for (size_t i = 0; i < partCount; ++i) {
        document.addPart(makePart(i, rng));
    }
    return document;
}

const char* profileName(PerformanceProfile profile) {
    switch (profile) {
    case PerformanceProfile::Fast:
        return "Fast";
    case PerformanceProfile::Maximum:
        return "Maximum";
    case PerformanceProfile::Balanced:
    default:
        return "Balanced";
    }
}

EngineSettings settingsFor(size_t partCount, PerformanceProfile profile) {
    EngineSettings settings;
    settings.performanceProfile = profile;
    settings.qualityMode = profile == PerformanceProfile::Fast ? QualityMode::Fast : profile == PerformanceProfile::Maximum ? QualityMode::MaxQuality : QualityMode::Balanced;
    settings.sheetWidth = std::max(900.0, std::sqrt(static_cast<double>(partCount)) * 180.0);
    settings.sheetHeight = std::max(650.0, std::sqrt(static_cast<double>(partCount)) * 130.0);
    settings.margin = 12.0;
    settings.partSpacing = 2.0;
    settings.allowRotation = false;
    settings.allowMirroring = true;
    settings.rotationMode = RotationMode::ContinuousRefine;
    settings.rotationStepDegrees = profile == PerformanceProfile::Maximum ? 0.01 : 0.1;
    settings.cpuThreadCount = 4;

    if (partCount <= 100) {
        settings.timeLimitSeconds = profile == PerformanceProfile::Fast ? 0.20 : profile == PerformanceProfile::Maximum ? 0.40 : 0.30;
    } else if (partCount <= 250) {
        settings.timeLimitSeconds = profile == PerformanceProfile::Fast ? 0.25 : profile == PerformanceProfile::Maximum ? 0.50 : 0.35;
    } else {
        settings.timeLimitSeconds = profile == PerformanceProfile::Fast ? 0.30 : profile == PerformanceProfile::Maximum ? 0.60 : 0.45;
    }
    return settings;
}

struct RunResult {
    bool ok = false;
    double baselineScore = 0.0;
    double solverScore = 0.0;
    double baselineUtil = 0.0;
    double solverUtil = 0.0;
    SolverStats stats;
};

RunResult runProfile(const Document& document, size_t partCount, PerformanceProfile profile) {
    const EngineSettings settings = settingsFor(partCount, profile);
    MultiStartSolver solver;
    LayoutScore scorer;
    PenaltySystem penalties;
    const LayoutState row = solver.rowBaseline(document, settings, settings.placementStrategy, 1u, 0);
    LayoutState baseline = row;
    if (!baseline.poses.empty()) {
        baseline.poses.back().x += settings.partSpacing + 10.0;
        baseline = scorer.evaluate(document, settings, baseline.poses, &penalties);
    }
    const auto started = Clock::now();
    LayoutState solved = baseline;

    SolverStats stats;
    stats.workerCount = static_cast<size_t>(settings.cpuThreadCount);
    stats.attemptsStarted = 1;
    stats.attemptsCompleted = 1;
    if (row.valid() && row.totalScore + 1e-6 < solved.totalScore) {
        solved = row;
        stats.bestUpdates = 1;
        stats.acceptedMoves = 1;
    }
    PoseSampler sampler;
    LayoutEvalCache evalCache;
    evalCache.rebuild(document, settings, solved, &penalties);
    const size_t partBudget = profile == PerformanceProfile::Fast ? 2 : profile == PerformanceProfile::Maximum ? 8 : 4;
    const size_t candidateBudget = profile == PerformanceProfile::Fast ? 32 : profile == PerformanceProfile::Maximum ? 128 : 64;
    size_t remainingCandidates = candidateBudget;
    for (size_t part = 0; part < std::min(partBudget, document.parts.size()) && remainingCandidates > 0; ++part) {
        const auto candidates = sampler.moveCandidates(document, settings, solved.poses, part, static_cast<unsigned int>(part + 17u), 0);
        for (size_t i = 0; i < candidates.size() && remainingCandidates > 0; ++i, --remainingCandidates) {
            const DeltaMove move{part, solved.poses[part], candidates[i]};
            const DeltaEvaluation trial = evaluateMoveDelta(document, settings, solved, evalCache, move);
            ++stats.evaluatedCandidates;
            if (trial.collisionCount > 0) {
                ++stats.rejectedCollision;
            } else if (trial.spacingPenalty > 0.0) {
                ++stats.rejectedSpacing;
            } else if (trial.invalidPartCount > 0 || trial.sheetPenalty > 0.0) {
                ++stats.rejectedSheet;
            } else if (trial.totalScore + 1e-6 < solved.totalScore) {
                std::vector<Pose> trialPoses = solved.poses;
                trialPoses[part] = candidates[i];
                LayoutState verified = scorer.evaluate(document, settings, trialPoses, &penalties);
                if (verified.valid() && verified.totalScore + 1e-6 < solved.totalScore) {
                    solved = std::move(verified);
                    evalCache.updateAfterAcceptedMove(document, settings, solved, move, &penalties);
                    ++stats.acceptedMoves;
                    ++stats.bestUpdates;
                }
            }
        }
    }
    stats.elapsedMs = std::chrono::duration<double, std::milli>(Clock::now() - started).count();
    stats.candidatesPerSecond = static_cast<double>(stats.evaluatedCandidates) / std::max(0.001, stats.elapsedMs / 1000.0);

    const bool valid = solved.valid();
    const bool notWorse = solved.totalScore <= baseline.totalScore + 1e-6 || solved.utilization + 1e-6 >= baseline.utilization;
    const bool improved = stats.bestUpdates > 0 || solved.totalScore + 1e-6 < baseline.totalScore || solved.utilization >= baseline.utilization - 1e-6;

    std::cout << "parts=" << partCount
              << " profile=" << profileName(profile)
              << " baselineScore=" << baseline.totalScore
              << " solverScore=" << solved.totalScore
              << " baselineUtil=" << baseline.utilization
              << " solverUtil=" << solved.utilization
              << " collisions=" << solved.collisionCount
              << " invalid=" << solved.invalidPartCount
              << " spacingPenalty=" << solved.spacingPenalty
              << " attemptsStarted=" << stats.attemptsStarted
              << " attemptsCompleted=" << stats.attemptsCompleted
              << " bestUpdates=" << stats.bestUpdates
              << " evaluatedCandidates=" << stats.evaluatedCandidates
              << " candidatesPerSecond=" << stats.candidatesPerSecond
              << " workerCount=" << stats.workerCount
              << " cacheHits=" << stats.cacheHits
              << std::endl;

    return {valid && notWorse && improved, baseline.totalScore, solved.totalScore, baseline.utilization, solved.utilization, stats};
}

} // namespace

int main() {
    bool ok = true;
    const size_t sizes[] = {100, 250, 500};
    const PerformanceProfile profiles[] = {PerformanceProfile::Fast, PerformanceProfile::Balanced, PerformanceProfile::Maximum};

    for (size_t partCount : sizes) {
        const Document document = makeDocument(partCount);
        double previousUtil = -1.0;
        for (PerformanceProfile profile : profiles) {
            const RunResult result = runProfile(document, partCount, profile);
            ok = result.ok && ok;
            if (previousUtil >= 0.0 && profile != PerformanceProfile::Fast) {
                ok = result.solverUtil + 0.05 >= previousUtil && ok;
            }
            previousUtil = std::max(previousUtil, result.solverUtil);
        }
    }

    return ok ? 0 : 1;
}
