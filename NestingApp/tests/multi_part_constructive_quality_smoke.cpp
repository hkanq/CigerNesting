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

EngineSettings settings() {
    EngineSettings s;
    s.sheetWidth = 1450.0;
    s.sheetHeight = 940.0;
    s.margin = 10.0;
    s.partSpacing = 2.0;
    s.performanceProfile = PerformanceProfile::Maximum;
    s.qualityMode = QualityMode::MaxQuality;
    s.rotationMode = RotationMode::None;
    s.allowRotation = false;
    s.allowMirroring = false;
    s.timeLimitSeconds = 0.0;
    s.cpuThreadCount = 1;
    s.deterministic = true;
    s.randomSeed = 102u;
    s.collisionTolerance = 0.01;
    s.curveFlattenTolerance = 0.35;
    return s;
}

Document loadMixed100(const std::filesystem::path& root, const EngineSettings& s) {
    Importer importer;
    ImportResult imported = importer.importFile((root / L"samples" / L"benchmark" / L"mixed_100_parts.svg").wstring(), s.curveFlattenTolerance);
    Document document;
    document.sheet.width = s.sheetWidth;
    document.sheet.height = s.sheetHeight;
    document.sheet.margin = s.margin;
    for (Part& part : imported.parts) {
        document.addPart(std::move(part));
    }
    return document;
}

bool valid(const LayoutState& state) {
    return state.collisionCount == 0 && state.invalidPartCount == 0 && state.spacingPenalty <= 1e-9 && state.sheetPenalty <= 1e-9;
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    if (argc < 2) {
        std::wcerr << L"usage: multi_part_constructive_quality_smoke <repo-root>\n";
        return 2;
    }
    EngineSettings s = settings();
    Document document = loadMixed100(argv[1], s);
    std::atomic_bool stop{false};
    MultiStartSolver solver;
    LayoutState solved = solver.solve(document, s, stop, {});
    PenaltySystem penalties;
    solved = LayoutScore{}.evaluate(document, s, solved.poses, &penalties);
    const SolverStats stats = solver.lastStats();

    std::cout << "real_constructive mixed_100"
              << " util=" << solved.utilization
              << " attempts=" << stats.destroyAttempts
              << " accepted=" << stats.destroyAccepted
              << " destroyBestUpdates=" << stats.destroyBestUpdates
              << " bestUpdates=" << stats.bestUpdates
              << " tempAcceptedObjective=" << stats.destroyTemporaryAcceptedWithObjectiveGain
              << " tempRejectedNoGain=" << stats.destroyTemporaryRejectedNoObjectiveGain
              << " compactionAttempts=" << stats.rebuildCompactionAttempts
              << " compactionClusters=" << stats.rebuildCompactionClusters
              << " compactionAccepted=" << stats.rebuildCompactionAccepted
              << " coordinatedAttempts=" << stats.coordinatedClusterRebuildAttempts
              << " coordinatedAccepted=" << stats.coordinatedClusterRebuildAccepted
              << " coordinatedMotion=" << stats.coordinatedClusterMotionAccepted
              << " avgCluster=" << stats.averageCoordinatedClusterSize
              << " denseAttempts=" << stats.denseSmallPartCompactionAttempts
              << " denseAccepted=" << stats.denseSmallPartCompactionAccepted
              << " avgSubset=" << stats.averageSubsetSize
              << " avgDepth=" << stats.averagePlacementDepth
              << " avgActiveContactDepth=" << stats.averageActiveContactDepth
              << " beamNodes=" << stats.beamNodesExpanded
              << " beamLeaves=" << stats.beamValidLeaves
              << " bestLargestReduction=" << stats.bestRebuildLargestEmptyRegionReduction
              << " bestUsedReduction=" << stats.bestRebuildUsedAreaReduction
              << " bestWidthReduction=" << stats.bestRebuildUsedWidthReduction
              << " bestHeightReduction=" << stats.bestRebuildUsedHeightReduction
              << " bestUtilGain=" << stats.bestRebuildUtilizationGain
              << " bestContactGain=" << stats.bestRebuildContactGain
              << "\n";

    bool ok = true;
    ok = expect("strict validity", valid(solved)) && ok;
    ok = expect("constructive attempts are real", stats.destroyAttempts >= 1) && ok;
    ok = expect("accepted rebuilds require objective gain", stats.destroyAccepted == stats.destroyBestUpdates + stats.destroyTemporaryAcceptedWithObjectiveGain) && ok;
    ok = expect("active contact depth is real", stats.averageActiveContactDepth >= 16.0) && ok;
    ok = expect("beam has valid leaves", stats.beamValidLeaves > 0) && ok;
    ok = expect("coordinated cluster rebuild ran", stats.coordinatedClusterRebuildAttempts > 0 && stats.averageCoordinatedClusterSize >= 8.0) && ok;
    ok = expect("constructive rebuild improves best or an objective",
        stats.bestRebuildUsedAreaReduction > 1.0 ||
        stats.bestRebuildUsedWidthReduction > 0.25 ||
        stats.bestRebuildUsedHeightReduction > 0.25 ||
        stats.bestRebuildUtilizationGain > 0.010) && ok;
    ok = expect("mixed_100 short quality floor", solved.utilization >= 0.52) && ok;
    return ok ? 0 : 1;
}
