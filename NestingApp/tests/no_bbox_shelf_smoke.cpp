#include "core/document.h"
#include "engine/layout_score.h"
#include "engine/multi_start_solver.h"
#include "engine/penalty_system.h"
#include "import/importer.h"
#include <atomic>
#include <filesystem>
#include <iostream>

namespace {

using namespace nest;

int fail(const char* message) {
    std::cout << "FAIL: " << message << "\n";
    return 1;
}

EngineSettings settingsForMixed500() {
    EngineSettings settings;
    settings.sheetWidth = 3300.0;
    settings.sheetHeight = 2150.0;
    settings.margin = 12.0;
    settings.partSpacing = 0.0;
    settings.allowRotation = false;
    settings.rotationMode = RotationMode::None;
    settings.allowMirroring = true;
    settings.performanceProfile = PerformanceProfile::Maximum;
    settings.qualityMode = QualityMode::MaxQuality;
    settings.timeLimitSeconds = 30.0;
    settings.cpuThreadCount = 1;
    settings.deterministic = true;
    settings.randomSeed = 104u;
    settings.collisionTolerance = 0.01;
    return settings;
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    if (argc < 2) {
        std::wcerr << L"usage: no_bbox_shelf_smoke <repo-root>\n";
        return 2;
    }
    const std::filesystem::path root = argv[1];
    const std::filesystem::path input = root / L"samples" / L"benchmark" / L"mixed_500_parts.svg";
    EngineSettings settings = settingsForMixed500();

    Importer importer;
    ImportResult imported = importer.importFile(input.wstring(), 0.35);
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

    std::atomic_bool stopRequested{false};
    MultiStartSolver solver;
    LayoutState row = solver.rowBaseline(document, settings, settings.placementStrategy, settings.randomSeed, 0);
    LayoutState solved = solver.solve(document, settings, stopRequested, {});
    PenaltySystem penalties;
    row = LayoutScore{}.evaluate(document, settings, row.poses, &penalties);
    solved = LayoutScore{}.evaluate(document, settings, solved.poses, &penalties);
    const SolverStats stats = solver.lastStats();
    std::cout << "parts=" << document.parts.size()
              << " valid=" << solved.valid()
              << " collision=" << solved.collisionCount
              << " invalid=" << solved.invalidPartCount
              << " spacingPenalty=" << solved.spacingPenalty
              << " sheetPenalty=" << solved.sheetPenalty
              << " rowUsed=" << stats.rowBaselineUsed
              << " rowFallback=" << stats.rowFallbackUsed
              << " contourSeed=" << stats.contourSeedUsed
              << " rowUtil=" << row.utilization
              << " util=" << solved.utilization
              << " contactReward=" << solved.contactReward
              << " contactTasks=" << stats.activeMoveSummary.contact
              << "\n";
    if (!solved.valid() ||
        solved.collisionCount != 0 ||
        solved.invalidPartCount != 0 ||
        solved.spacingPenalty > 0.0 ||
        solved.sheetPenalty > 0.0) {
        return fail("Maximum mixed_500 result is not fully valid");
    }
    if (stats.rowBaselineUsed != 0 || stats.rowFallbackUsed != 0 || stats.contourSeedUsed == 0) {
        return fail("Maximum profile used row/shelf baseline");
    }
    if (solved.utilization + 1e-6 < row.utilization || solved.contactReward <= 0.0 || stats.activeMoveSummary.contact == 0) {
        return fail("contour-contact path did not beat the row/shelf baseline");
    }
    std::cout << "PASS: no bbox shelf smoke\n";
    return 0;
}
