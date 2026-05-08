#include "core/document.h"
#include "engine/empty_space_map.h"
#include "engine/layout_score.h"
#include "engine/multi_start_solver.h"
#include "engine/penalty_system.h"
#include "import/importer.h"

#include <atomic>
#include <filesystem>
#include <iostream>
#include <string>
#include <utility>

namespace {

using namespace nest;

struct QualityCase {
    const wchar_t* fileName = L"";
    const char* name = "";
    double sheetWidth = 1000.0;
    double sheetHeight = 700.0;
    double margin = 8.0;
    double spacing = 2.0;
    uint32_t seed = 1u;
    double minimumUtilization = 0.65;
    double timeLimitSeconds = 30.0;
};

bool expect(const char* name, bool condition) {
    std::cout << (condition ? "PASS: " : "QUALITY_FAIL: ") << name << "\n";
    return condition;
}

EngineSettings settingsFor(const QualityCase& testCase) {
    EngineSettings settings;
    settings.sheetWidth = testCase.sheetWidth;
    settings.sheetHeight = testCase.sheetHeight;
    settings.margin = testCase.margin;
    settings.partSpacing = testCase.spacing;
    settings.allowRotation = false;
    settings.allowMirroring = false;
    settings.rotationMode = RotationMode::None;
    settings.performanceProfile = PerformanceProfile::Maximum;
    settings.qualityMode = QualityMode::MaxQuality;
    settings.timeLimitSeconds = testCase.timeLimitSeconds;
    settings.cpuThreadCount = 1;
    settings.deterministic = true;
    settings.randomSeed = testCase.seed;
    settings.collisionTolerance = 0.01;
    settings.curveFlattenTolerance = 0.35;
    return settings;
}

Document loadDocument(const std::filesystem::path& root, const QualityCase& testCase, const EngineSettings& settings) {
    Importer importer;
    const std::filesystem::path input = root / L"samples" / L"benchmark" / testCase.fileName;
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

struct RunResult {
    LayoutState state;
    SolverStats stats;
    EmptySpaceMap emptyMap;
};

RunResult runCase(const std::filesystem::path& root, const QualityCase& testCase) {
    EngineSettings settings = settingsFor(testCase);
    Document document = loadDocument(root, testCase, settings);
    std::atomic_bool stopRequested{false};
    MultiStartSolver solver;
    LayoutState state = solver.solve(document, settings, stopRequested, {});
    PenaltySystem penalties;
    state = LayoutScore{}.evaluate(document, settings, state.poses, &penalties);
    EmptySpaceMap emptyMap = EmptySpaceAnalyzer{}.analyze(document, settings, state);
    return {std::move(state), solver.lastStats(), std::move(emptyMap)};
}

bool validLayout(const LayoutState& state) {
    return state.collisionCount == 0 &&
        state.invalidPartCount == 0 &&
        state.spacingPenalty <= 1e-9 &&
        state.sheetPenalty <= 1e-9;
}

bool runQualityGate(const std::filesystem::path& root, const QualityCase& testCase) {
    const RunResult result = runCase(root, testCase);
    std::cout << testCase.name
              << " util=" << result.state.utilization
              << " collisions=" << result.state.collisionCount
              << " invalid=" << result.state.invalidPartCount
              << " spacingPenalty=" << result.state.spacingPenalty
              << " sheetPenalty=" << result.state.sheetPenalty
              << " bestUpdates=" << result.stats.bestUpdates
              << " destroyBestUpdates=" << result.stats.destroyBestUpdates
              << " compactionAttempts=" << result.stats.rebuildCompactionAttempts
              << " compactionClusters=" << result.stats.rebuildCompactionClusters
              << " compactionAccepted=" << result.stats.rebuildCompactionAccepted
              << " coordinatedAttempts=" << result.stats.coordinatedClusterRebuildAttempts
              << " coordinatedAccepted=" << result.stats.coordinatedClusterRebuildAccepted
              << " coordinatedMotion=" << result.stats.coordinatedClusterMotionAccepted
              << " avgCluster=" << result.stats.averageCoordinatedClusterSize
              << " denseAttempts=" << result.stats.denseSmallPartCompactionAttempts
              << " denseAccepted=" << result.stats.denseSmallPartCompactionAccepted
              << " bestUsedReduction=" << result.stats.bestRebuildUsedAreaReduction
              << " bestWidthReduction=" << result.stats.bestRebuildUsedWidthReduction
              << " bestHeightReduction=" << result.stats.bestRebuildUsedHeightReduction
              << " bestUtilGain=" << result.stats.bestRebuildUtilizationGain
              << " emptySpace=" << result.emptyMap.totalEmptyArea
              << " largestGap=" << result.emptyMap.largestRegionArea
              << " fillableGaps=" << result.stats.fillableGapCount
              << " contactCount=" << result.stats.contactCount
              << " towerScore=" << result.stats.towerScore
              << " lowContact=" << result.stats.lowContactPartCount
              << " slideAccepted=" << result.stats.slideToContactAccepted
              << " aggressiveGapAccepted=" << result.stats.aggressiveGapAccepted
              << " localRegionRepackAccepted=" << result.stats.localRegionRepackAccepted
              << "\n";
    bool ok = true;
    ok = expect("final layout validity is strict", validLayout(result.state)) && ok;
    ok = expect("constructive compaction produced a real used-area/utilization signal",
        result.stats.bestRebuildUsedAreaReduction > 1.0 ||
        result.stats.bestRebuildUsedWidthReduction > 0.25 ||
        result.stats.bestRebuildUsedHeightReduction > 0.25 ||
        result.stats.bestRebuildUtilizationGain > 0.010) && ok;
    ok = expect("Maximum utilization target", result.state.utilization + 1e-9 >= testCase.minimumUtilization) && ok;
    ok = expect("empty space map reports free regions", !result.emptyMap.regions.empty() && result.emptyMap.largestRegionArea > 0.0) && ok;
    return ok;
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    if (argc < 2) {
        std::wcerr << L"usage: industrial_quality_smoke <repo-root>\n";
        return 2;
    }

    const std::filesystem::path root = argv[1];
    bool ok = true;
    ok = runQualityGate(root, {L"mixed_500_parts.svg", "mixed_500_parts", 3300.0, 2150.0, 12.0, 2.0, 104u, 0.65, 30.0}) && ok;
    ok = runQualityGate(root, {L"many_small_parts.svg", "many_small_parts", 1150.0, 760.0, 8.0, 2.0, 101u, 0.65, 18.0}) && ok;
    ok = runQualityGate(root, {L"mixed_100_parts.svg", "mixed_100_parts", 1450.0, 940.0, 10.0, 2.0, 102u, 0.72, 18.0}) && ok;
    return ok ? 0 : 1;
}
