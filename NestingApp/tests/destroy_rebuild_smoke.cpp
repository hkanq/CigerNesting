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
              << " destroyTemporaryAcceptedWithObjectiveGain=" << stats.destroyTemporaryAcceptedWithObjectiveGain
              << " destroyTemporaryRejectedNoObjectiveGain=" << stats.destroyTemporaryRejectedNoObjectiveGain
              << " destroyBestUpdates=" << stats.destroyBestUpdates
              << " bestUpdates=" << stats.bestUpdates
              << " compactionAttempts=" << stats.rebuildCompactionAttempts
              << " compactionClusters=" << stats.rebuildCompactionClusters
              << " compactionAccepted=" << stats.rebuildCompactionAccepted
              << " coordinatedAttempts=" << stats.coordinatedClusterRebuildAttempts
              << " coordinatedAccepted=" << stats.coordinatedClusterRebuildAccepted
              << " coordinatedMotion=" << stats.coordinatedClusterMotionAccepted
              << " avgCluster=" << stats.averageCoordinatedClusterSize
              << " denseAttempts=" << stats.denseSmallPartCompactionAttempts
              << " denseAccepted=" << stats.denseSmallPartCompactionAccepted
              << " averageSubsetSize=" << stats.averageSubsetSize
              << " beamNodesExpanded=" << stats.beamNodesExpanded
              << " beamValidLeaves=" << stats.beamValidLeaves
              << " beforeLargest=" << stats.rebuildBeforeLargestEmptyRegion
              << " afterLargest=" << stats.rebuildAfterLargestEmptyRegion
              << " beforeUsed=" << stats.rebuildBeforeUsedArea
              << " afterUsed=" << stats.rebuildAfterUsedArea
              << " beforeContact=" << stats.rebuildBeforeContactCount
              << " afterContact=" << stats.rebuildAfterContactCount
              << " bestLargestReduction=" << stats.bestRebuildLargestEmptyRegionReduction
              << " bestUsedReduction=" << stats.bestRebuildUsedAreaReduction
              << " bestWidthReduction=" << stats.bestRebuildUsedWidthReduction
              << " bestHeightReduction=" << stats.bestRebuildUsedHeightReduction
              << " bestUtilGain=" << stats.bestRebuildUtilizationGain
              << " bestContactGain=" << stats.bestRebuildContactGain
              << " acceptedBetter=" << stats.acceptedBetter
              << " acceptedTemporary=" << stats.acceptedTemporary
              << " acceptanceRate=" << stats.acceptanceRate
              << "\n";

    bool ok = true;
    ok = expect("strict final validity", validLayout(solved)) && ok;
    ok = expect("destroy-rebuild attempted", stats.destroyAttempts > 0) && ok;
    ok = expect("destroy-rebuild expanded beam nodes", stats.beamNodesExpanded > 0) && ok;
    ok = expect("valid-only temporary acceptance is blocked",
        stats.destroyAccepted == stats.destroyBestUpdates + stats.destroyTemporaryAcceptedWithObjectiveGain) && ok;
    ok = expect("post-rebuild compaction ran", stats.rebuildCompactionAttempts > 0 && stats.rebuildCompactionClusters > 0) && ok;
    ok = expect("coordinated cluster rebuild ran", stats.coordinatedClusterRebuildAttempts > 0 && stats.averageCoordinatedClusterSize >= 8.0) && ok;
    ok = expect("destroy accepted is not used as a fake success",
        stats.bestRebuildUsedAreaReduction > 1.0 ||
        stats.bestRebuildUsedWidthReduction > 0.25 ||
        stats.bestRebuildUsedHeightReduction > 0.25 ||
        stats.bestRebuildUtilizationGain > 0.010) && ok;
    ok = expect("mixed_100 remains above current validity baseline; industrial benchmark owns utilization gate", solved.utilization >= 0.52) && ok;
    return ok ? 0 : 1;
}
