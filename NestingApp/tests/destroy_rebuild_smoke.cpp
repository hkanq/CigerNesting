#include "core/document.h"
#include "engine/layout_score.h"
#include "engine/multi_start_solver.h"
#include "engine/penalty_system.h"
#include "import/importer.h"

#include <atomic>
#include <filesystem>
#include <iostream>
#include <utility>

namespace {

using namespace nest;

bool expect(const char* name, bool condition) {
    std::cout << (condition ? "PASS: " : "QUALITY_FAIL: ") << name << "\n";
    return condition;
}

EngineSettings makeSettings() {
    EngineSettings settings;
    settings.sheetWidth = 1450.0;
    settings.sheetHeight = 940.0;
    settings.margin = 10.0;
    settings.partSpacing = 2.0;
    settings.performanceProfile = PerformanceProfile::Maximum;
    settings.qualityMode = QualityMode::MaxQuality;
    settings.rotationMode = RotationMode::None;
    settings.allowRotation = false;
    settings.allowMirroring = false;
    settings.timeLimitSeconds = 18.0;
    settings.cpuThreadCount = 1;
    settings.deterministic = true;
    settings.randomSeed = 102u;
    settings.collisionTolerance = 0.01;
    settings.curveFlattenTolerance = 0.35;
    return settings;
}

Document loadDocument(const std::filesystem::path& root, const EngineSettings& settings) {
    Importer importer;
    const std::filesystem::path input = root / L"samples" / L"benchmark" / L"mixed_100_parts.svg";
    ImportResult imported = importer.importFile(input.wstring(), settings.curveFlattenTolerance);
    Document document;
    document.sheet.width = settings.sheetWidth;
    document.sheet.height = settings.sheetHeight;
    document.sheet.margin = settings.margin;
    if (!imported.ok) {
        std::wcerr << L"FAIL: import failed " << input << L"\n";
        return document;
    }
    for (Part& part : imported.parts) {
        document.addPart(std::move(part));
    }
    return document;
}

bool validLayout(const LayoutState& state) {
    return state.collisionCount == 0 &&
        state.invalidPartCount == 0 &&
        state.spacingPenalty <= 1e-9 &&
        state.sheetPenalty <= 1e-9;
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    if (argc < 2) {
        std::wcerr << L"usage: destroy_rebuild_smoke <repo-root>\n";
        return 2;
    }

    const std::filesystem::path root = argv[1];
    EngineSettings settings = makeSettings();
    Document document = loadDocument(root, settings);
    std::atomic_bool stopRequested{false};
    MultiStartSolver solver;
    LayoutState solved = solver.solve(document, settings, stopRequested, {});
    PenaltySystem penalties;
    solved = LayoutScore{}.evaluate(document, settings, solved.poses, &penalties);
    const SolverStats stats = solver.lastStats();

    std::cout << "mixed_100 destroy-rebuild"
              << " util=" << solved.utilization
              << " collision=" << solved.collisionCount
              << " invalid=" << solved.invalidPartCount
              << " spacingPenalty=" << solved.spacingPenalty
              << " sheetPenalty=" << solved.sheetPenalty
              << " destroyAttempts=" << stats.destroyAttempts
              << " destroyAccepted=" << stats.destroyAccepted
              << " destroyTemporaryAccepted=" << stats.destroyTemporaryAccepted
              << " destroyBestUpdates=" << stats.destroyBestUpdates
              << " averageSubsetSize=" << stats.averageSubsetSize
              << " beamNodesExpanded=" << stats.beamNodesExpanded
              << " beamValidLeaves=" << stats.beamValidLeaves
              << " acceptedBetter=" << stats.acceptedBetter
              << " acceptedTemporary=" << stats.acceptedTemporary
              << " acceptanceRate=" << stats.acceptanceRate
              << "\n";

    bool ok = true;
    ok = expect("strict final validity", validLayout(solved)) && ok;
    ok = expect("destroy-rebuild attempted", stats.destroyAttempts > 0) && ok;
    ok = expect("destroy-rebuild expanded beam nodes", stats.beamNodesExpanded > 0) && ok;
    ok = expect("destroy-rebuild accepted at least one trajectory", stats.destroyAccepted > 0) && ok;
    ok = expect("mixed_100 utilization moved beyond local-only result", solved.utilization > 0.547) && ok;
    return ok ? 0 : 1;
}
