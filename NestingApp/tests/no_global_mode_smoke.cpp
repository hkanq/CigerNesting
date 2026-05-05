#include "core/document.h"
#include "engine/layout_score.h"
#include "engine/multi_start_solver.h"
#include "engine/penalty_system.h"
#include <atomic>
#include <iostream>
#include <utility>
#include <vector>

namespace {

using namespace nest;

Ring boxRing(double x0, double y0, double x1, double y1) {
    Ring ring;
    ring.points = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}, {x0, y0}};
    return ring;
}

Part partFromRings(std::vector<Ring> rings) {
    Part part;
    part.rings = std::move(rings);
    part.updateDerivedGeometry();
    return part;
}

Part boxPart(double w, double h) {
    return partFromRings({boxRing(0.0, 0.0, w, h)});
}

Part donutPart() {
    Ring hole = boxRing(34.0, 34.0, 76.0, 76.0);
    hole.isHole = true;
    return partFromRings({boxRing(0.0, 0.0, 110.0, 110.0), hole});
}

Part cShapePart() {
    Ring ring;
    ring.points = {
        {0.0, 0.0}, {110.0, 0.0}, {110.0, 22.0}, {34.0, 22.0},
        {34.0, 64.0}, {110.0, 64.0}, {110.0, 86.0}, {0.0, 86.0}, {0.0, 0.0}
    };
    return partFromRings({ring});
}

size_t nonZeroGroups(const ActiveMoveSummary& summary) {
    const size_t values[] = {
        summary.contact, summary.compression, summary.gap, summary.hole,
        summary.concavity, summary.smallPart, summary.swap, summary.chain,
        summary.cluster, summary.region, summary.rotation, summary.mirror,
        summary.escape, summary.frontier
    };
    size_t count = 0;
    for (size_t value : values) {
        if (value > 0) {
            ++count;
        }
    }
    return count;
}

int fail(const char* message) {
    std::cout << "FAIL: " << message << "\n";
    return 1;
}

} // namespace

int main() {
    Document document;
    document.sheet.width = 420.0;
    document.sheet.height = 300.0;
    document.sheet.margin = 6.0;
    document.addPart(donutPart());
    document.addPart(cShapePart());
    document.addPart(boxPart(18.0, 18.0));
    document.addPart(boxPart(16.0, 16.0));
    document.addPart(boxPart(42.0, 12.0));
    document.addPart(boxPart(12.0, 44.0));
    for (int i = 0; i < 18; ++i) {
        document.addPart(boxPart(9.0 + static_cast<double>(i % 4), 8.0 + static_cast<double>(i % 3)));
    }

    EngineSettings settings;
    settings.sheetWidth = document.sheet.width;
    settings.sheetHeight = document.sheet.height;
    settings.margin = document.sheet.margin;
    settings.partSpacing = 0.0;
    settings.allowRotation = true;
    settings.rotationMode = RotationMode::ContinuousRefine;
    settings.allowMirroring = true;
    settings.performanceProfile = PerformanceProfile::Maximum;
    settings.qualityMode = QualityMode::MaxQuality;
    settings.timeLimitSeconds = 6.0;
    settings.cpuThreadCount = 1;
    settings.deterministic = true;
    settings.randomSeed = 911u;

    std::atomic_bool stopRequested{false};
    MultiStartSolver solver;
    LayoutState solved = solver.solve(document, settings, stopRequested, {});
    PenaltySystem penalties;
    solved = LayoutScore{}.evaluate(document, settings, solved.poses, &penalties);
    const SolverStats stats = solver.lastStats();
    const size_t groups = nonZeroGroups(stats.activeMoveSummary);
    std::cout << "valid=" << solved.valid()
              << " groups=" << groups
              << " contact=" << stats.activeMoveSummary.contact
              << " compression=" << stats.activeMoveSummary.compression
              << " hole=" << stats.activeMoveSummary.hole
              << " rotation=" << stats.activeMoveSummary.rotation
              << " swap=" << stats.activeMoveSummary.swap
              << " contourSeed=" << stats.contourSeedUsed
              << " row=" << stats.rowBaselineUsed
              << " rowFallback=" << stats.rowFallbackUsed
              << "\n";
    if (!solved.valid()) {
        return fail("solver returned invalid layout");
    }
    if (groups < 3 || stats.activeMoveSummary.contact == 0 || stats.activeMoveSummary.hole == 0) {
        return fail("optimizer did not evaluate a mixed per-part operator distribution");
    }
    if (stats.rowBaselineUsed != 0 || stats.rowFallbackUsed != 0 || stats.contourSeedUsed == 0) {
        return fail("Maximum profile fell back to row/shelf baseline");
    }
    std::cout << "PASS: no global mode smoke\n";
    return 0;
}
