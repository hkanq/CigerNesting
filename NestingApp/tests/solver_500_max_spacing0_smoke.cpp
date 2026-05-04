#include "core/document.h"
#include "engine/layout_score.h"
#include "engine/multi_start_solver.h"
#include "engine/penalty_system.h"
#include "import/importer.h"
#include <atomic>
#include <chrono>
#include <filesystem>
#include <iostream>

namespace {

using namespace nest;

int fail(const char* message) {
    std::cout << "FAIL: " << message << "\n";
    return 1;
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    if (argc < 2) {
        std::wcerr << L"usage: solver_500_max_spacing0_smoke <repo-root>\n";
        return 2;
    }

    const std::filesystem::path root = argv[1];
    const std::filesystem::path input = root / L"samples" / L"benchmark" / L"mixed_500_parts.svg";

    EngineSettings settings;
    settings.sheetWidth = 3300.0;
    settings.sheetHeight = 2150.0;
    settings.margin = 12.0;
    settings.partSpacing = 0.0;
    settings.allowRotation = false;
    settings.rotationMode = RotationMode::None;
    settings.allowMirroring = true;
    settings.qualityMode = QualityMode::MaxQuality;
    settings.performanceProfile = PerformanceProfile::Maximum;
    settings.timeLimitSeconds = 30.0;
    settings.cpuThreadCount = 1;
    settings.livePreviewIntervalMs = 250;
    settings.collisionTolerance = 0.01;
    settings.curveFlattenTolerance = 0.35;
    settings.randomSeed = 104u;
    settings.deterministic = true;

    Importer importer;
    ImportResult imported = importer.importFile(input.wstring(), settings.curveFlattenTolerance);
    if (!imported.ok) {
        std::wcerr << L"FAIL: import failed " << input << L"\n";
        return 1;
    }

    Document document;
    document.sheet.width = settings.sheetWidth;
    document.sheet.height = settings.sheetHeight;
    document.sheet.margin = settings.margin;
    for (Part& part : imported.parts) {
        document.addPart(std::move(part));
    }
    if (document.parts.size() != 500) {
        return fail("expected 500 imported parts");
    }

    std::atomic_bool stopRequested{false};
    MultiStartSolver solver;
    const auto solveStart = std::chrono::steady_clock::now();
    LayoutState solved = solver.solve(document, settings, stopRequested, {});
    const auto solveEnd = std::chrono::steady_clock::now();
    PenaltySystem penalties;
    solved = LayoutScore{}.evaluate(document, settings, solved.poses, &penalties);
    const auto evalEnd = std::chrono::steady_clock::now();
    const double solveMs = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(solveEnd - solveStart).count());
    const double evalMs = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(evalEnd - solveEnd).count());

    std::cout << "parts=" << document.parts.size()
              << " collision=" << solved.collisionCount
              << " invalid=" << solved.invalidPartCount
              << " spacingPenalty=" << solved.spacingPenalty
              << " sheetPenalty=" << solved.sheetPenalty
              << " utilization=" << solved.utilization
              << " bestUpdates=" << solver.lastStats().bestUpdates
              << " solveMs=" << solveMs
              << " finalEvalMs=" << evalMs
              << "\n";

    if (!solved.valid()) {
        return fail("final solver result is not valid");
    }
    if (solved.collisionCount != 0 || solved.invalidPartCount != 0 || solved.spacingPenalty > 0.0 || solved.sheetPenalty > 0.0) {
        return fail("validity counters are not zero");
    }
    return 0;
}
