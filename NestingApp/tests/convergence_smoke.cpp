#include "core/document.h"
#include "engine/layout_score.h"
#include "engine/multi_start_solver.h"
#include "engine/nesting_engine.h"
#include "engine/penalty_system.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

namespace {

using namespace nest;
using Clock = std::chrono::steady_clock;

Ring boxRing(double x0, double y0, double x1, double y1) {
    Ring ring;
    ring.points = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}, {x0, y0}};
    return ring;
}

Part boxPart(double w, double h) {
    Part part;
    part.rings.push_back(boxRing(0.0, 0.0, w, h));
    part.updateDerivedGeometry();
    return part;
}

Document makeDocument() {
    Document document;
    document.sheet.width = 300.0;
    document.sheet.height = 180.0;
    document.sheet.margin = 8.0;
    for (int i = 0; i < 18; ++i) {
        document.addPart(boxPart(18.0 + static_cast<double>(i % 4), 12.0 + static_cast<double>(i % 3)));
    }
    return document;
}

} // namespace

int main() {
    Document document = makeDocument();
    EngineSettings settings;
    settings.sheetWidth = document.sheet.width;
    settings.sheetHeight = document.sheet.height;
    settings.margin = document.sheet.margin;
    settings.partSpacing = 2.0;
    settings.performanceProfile = PerformanceProfile::Balanced;
    settings.qualityMode = QualityMode::Balanced;
    settings.timeLimitSeconds = 0.0;
    settings.cpuThreadCount = 1;
    settings.deterministic = true;
    settings.randomSeed = 77u;

    std::atomic_bool stopRequested{false};
    MultiStartSolver solver;
    const auto started = Clock::now();
    LayoutState solved = solver.solve(document, settings, stopRequested, {});
    const double elapsed = std::chrono::duration<double>(Clock::now() - started).count();
    PenaltySystem penalties;
    solved = LayoutScore{}.evaluate(document, settings, solved.poses, &penalties);

    std::cout << "autoElapsed=" << elapsed
              << " valid=" << solved.valid()
              << " collision=" << solved.collisionCount
              << " invalid=" << solved.invalidPartCount
              << " spacingPenalty=" << solved.spacingPenalty
              << " sheetPenalty=" << solved.sheetPenalty
              << " attemptsCompleted=" << solver.lastStats().attemptsCompleted
              << "\n";

    if (elapsed > 20.0) {
        std::cout << "FAIL: auto convergence took too long\n";
        return 1;
    }
    if (!solved.valid()) {
        std::cout << "FAIL: auto convergence returned invalid layout\n";
        return 1;
    }
    if (solver.lastStats().attemptsCompleted == 0) {
        std::cout << "FAIL: solver completed no attempts\n";
        return 1;
    }

    Document engineDocument = makeDocument();
    NestingEngine engine;
    engine.setDocument(&engineDocument);
    engine.setSettings(settings);
    engine.start();
    const auto engineStarted = Clock::now();
    while (engine.isRunning()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (std::chrono::duration<double>(Clock::now() - engineStarted).count() > 20.0) {
            engine.stop();
            std::cout << "FAIL: engine auto convergence did not stop\n";
            return 1;
        }
    }
    const SolverSnapshot snapshot = engine.getLatestSnapshot();
    const SolverResult result = engine.getBestResult();
    std::cout << "enginePhase=" << static_cast<int>(snapshot.phase)
              << " engineCollision=" << snapshot.collisionCount
              << " resultValid=" << result.valid << "\n";
    if (snapshot.phase != SolverPhase::Done || snapshot.collisionCount != 0 || !result.valid) {
        std::cout << "FAIL: engine Done snapshot is not the best valid layout\n";
        return 1;
    }
    std::cout << "PASS: auto convergence returns a valid layout\n";
    return 0;
}
