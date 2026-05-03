#include "core/document.h"
#include "core/math_utils.h"
#include "engine/layout_score.h"
#include "engine/penalty_system.h"
#include "engine/pose_sampler.h"
#include "engine/ultra_refinement.h"
#include "engine/worker_pool.h"
#include <atomic>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace nest;

Ring boxRing(double x0, double y0, double x1, double y1) {
    Ring ring;
    ring.points = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}, {x0, y0}};
    return ring;
}

Part partFromRing(Ring ring) {
    Part part;
    part.rings.push_back(std::move(ring));
    part.updateDerivedGeometry();
    return part;
}

Part boxPart(double x0, double y0, double x1, double y1) {
    return partFromRing(boxRing(x0, y0, x1, y1));
}

Vec2 rotated(Vec2 point, double degrees) {
    const double radians = degreesToRadians(degrees);
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    return {point.x * c - point.y * s, point.x * s + point.y * c};
}

Part bakedRotatedRectangle(double width, double height, double bakedDegrees) {
    const double hx = width * 0.5;
    const double hy = height * 0.5;
    Ring ring;
    ring.points = {
        rotated({-hx, -hy}, bakedDegrees),
        rotated({ hx, -hy}, bakedDegrees),
        rotated({ hx,  hy}, bakedDegrees),
        rotated({-hx,  hy}, bakedDegrees),
        rotated({-hx, -hy}, bakedDegrees)
    };
    return partFromRing(std::move(ring));
}

Part hookPart() {
    Ring ring;
    ring.points = {
        {0.0, 0.0}, {80.0, 0.0}, {80.0, 18.0}, {34.0, 18.0},
        {34.0, 58.0}, {0.0, 58.0}, {0.0, 0.0}
    };
    return partFromRing(std::move(ring));
}

EngineSettings baseSettings() {
    EngineSettings settings;
    settings.sheetWidth = 320.0;
    settings.sheetHeight = 180.0;
    settings.margin = 8.0;
    settings.partSpacing = 3.0;
    settings.allowRotation = true;
    settings.rotationMode = RotationMode::ContinuousRefine;
    settings.rotationStepDegrees = 0.001;
    settings.qualityMode = QualityMode::MaxQuality;
    settings.timeLimitSeconds = 2.0;
    settings.cpuThreadCount = 4;
    return settings;
}

LayoutState evaluatedState(const Document& document, const EngineSettings& settings, std::vector<Pose> poses) {
    LayoutScore scorer;
    PenaltySystem penalties;
    return scorer.evaluate(document, settings, poses, &penalties);
}

struct ScenarioResult {
    bool passed = false;
    LayoutState before;
    LayoutState after;
    UltraRefinementStats stats;
};

void printScenario(const char* name, const ScenarioResult& result) {
    std::cout << name
              << " beforeScore=" << result.before.totalScore
              << " afterScore=" << result.after.totalScore
              << " beforeUtil=" << result.before.utilization
              << " afterUtil=" << result.after.utilization
              << " acceptedMoves=" << result.stats.acceptedMoves
              << " angleStepMinUsed=" << result.stats.angleStepMinUsedDegrees
              << " evaluatedCandidates=" << result.stats.evaluatedCandidates
              << " collisions=" << result.after.collisionCount
              << " invalid=" << result.after.invalidPartCount << "\n";
}

ScenarioResult refineSingleRotatedPart(double startAngleDegrees) {
    EngineSettings settings = baseSettings();
    Document document;
    document.sheet.width = settings.sheetWidth;
    document.sheet.height = settings.sheetHeight;
    document.sheet.margin = settings.margin;
    document.addPart(bakedRotatedRectangle(130.0, 18.0, 37.0));

    LayoutState state;
    state.poses.resize(1);
    state.poses[0].x = 160.0;
    state.poses[0].y = 90.0;
    state.poses[0].angleRadians = degreesToRadians(startAngleDegrees);

    PenaltySystem penalties;
    LayoutScore scorer;
    state = scorer.evaluate(document, settings, state.poses, &penalties);

    WorkerPool pool(4);
    std::atomic_bool stop{false};
    UltraRefinement refinement;
    const LayoutState refined = refinement.refine(document, settings, state, penalties, pool, stop);
    const UltraRefinementStats stats = refinement.lastStats();
    const double startDistance = std::abs(startAngleDegrees + 37.0);
    const double endDistance = std::abs(radiansToDegrees(refined.poses[0].angleRadians) + 37.0);

    return {
        refined.valid() &&
            refined.totalScore + 1e-6 < state.totalScore &&
            endDistance + 1e-6 < startDistance &&
            stats.acceptedMoves > 0,
        state,
        refined,
        stats
    };
}

ScenarioResult mirrorImprovesLayout() {
    EngineSettings settings = baseSettings();
    settings.allowRotation = false;
    settings.allowMirroring = true;
    settings.qualityMode = QualityMode::Balanced;
    settings.timeLimitSeconds = 1.0;

    Document document;
    document.sheet.width = settings.sheetWidth;
    document.sheet.height = settings.sheetHeight;
    document.sheet.margin = settings.margin;
    document.addPart(boxPart(0.0, 0.0, 30.0, 30.0));
    document.addPart(hookPart());

    LayoutState state;
    state.poses.resize(2);
    state.poses[0].x = 20.0;
    state.poses[0].y = 30.0;
    state.poses[1].x = 190.0;
    state.poses[1].y = 30.0;
    state.poses[1].mirrored = false;

    PenaltySystem penalties;
    LayoutScore scorer;
    state = scorer.evaluate(document, settings, state.poses, &penalties);

    WorkerPool pool(4);
    std::atomic_bool stop{false};
    UltraRefinement refinement;
    const LayoutState refined = refinement.refine(document, settings, state, penalties, pool, stop);
    const UltraRefinementStats stats = refinement.lastStats();

    return {
        refined.valid() &&
            refined.totalScore + 1e-6 < state.totalScore &&
            refined.poses.size() > 1 &&
            refined.poses[1].mirrored &&
            stats.acceptedMoves > 0,
        state,
        refined,
        stats
    };
}

bool bruteForceGuard(const ScenarioResult& result) {
    return result.stats.angleStepMinUsedDegrees <= 0.001 + 1e-12 &&
           result.stats.evaluatedCandidates > 0 &&
           result.stats.evaluatedCandidates < 360000;
}

bool fixedStepCoarseSamplingIsCapped() {
    EngineSettings settings = baseSettings();
    settings.rotationMode = RotationMode::FixedStep;
    settings.rotationStepDegrees = 0.001;
    const std::vector<double> samples = PoseSampler{}.coarseRotationSamples(settings);
    std::cout << "fixed_step_0_001_coarseSamples=" << samples.size() << "\n";
    return !samples.empty() && samples.size() < 360000;
}

} // namespace

int main() {
    const ScenarioResult longThin = refineSingleRotatedPart(-36.35);
    printScenario("long_thin_rotation", longThin);

    const ScenarioResult nearThirtySeven = refineSingleRotatedPart(-42.0);
    printScenario("near_37_degree_shape", nearThirtySeven);

    const ScenarioResult mirrored = mirrorImprovesLayout();
    printScenario("mirror_enabled", mirrored);

    bool ok = true;
    ok = longThin.passed && ok;
    ok = nearThirtySeven.passed && ok;
    ok = mirrored.passed && ok;
    ok = bruteForceGuard(longThin) && ok;
    ok = longThin.after.collisionCount == 0 && longThin.after.invalidPartCount == 0 && ok;
    ok = nearThirtySeven.after.collisionCount == 0 && nearThirtySeven.after.invalidPartCount == 0 && ok;
    ok = mirrored.after.collisionCount == 0 && mirrored.after.invalidPartCount == 0 && ok;
    ok = fixedStepCoarseSamplingIsCapped() && ok;

    std::cout << "bruteforce_guard=" << bruteForceGuard(longThin) << "\n";
    return ok ? 0 : 1;
}
