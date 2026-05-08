#pragma once

#include "core/document.h"
#include "engine/empty_space_map.h"
#include "engine/layout_score.h"
#include "engine/multi_start_solver.h"
#include "engine/penalty_system.h"
#include "import/importer.h"

#include <atomic>
#include <filesystem>
#include <iostream>
#include <utility>

namespace nest::test_smoke {

inline bool expect(const char* name, bool condition) {
    std::cout << (condition ? "PASS: " : "QUALITY_FAIL: ") << name << "\n";
    return condition;
}

inline EngineSettings maximumSettings() {
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
    settings.timeLimitSeconds = 0.0;
    settings.cpuThreadCount = 1;
    settings.deterministic = true;
    settings.randomSeed = 102u;
    settings.collisionTolerance = 0.01;
    settings.curveFlattenTolerance = 0.35;
    return settings;
}

inline Document loadBenchmark(const std::filesystem::path& root, const wchar_t* fileName, const EngineSettings& settings) {
    Importer importer;
    ImportResult imported = importer.importFile((root / L"samples" / L"benchmark" / fileName).wstring(), settings.curveFlattenTolerance);
    Document document;
    document.sheet.width = settings.sheetWidth;
    document.sheet.height = settings.sheetHeight;
    document.sheet.margin = settings.margin;
    for (Part& part : imported.parts) {
        document.addPart(std::move(part));
    }
    return document;
}

inline bool valid(const LayoutState& state) {
    return state.collisionCount == 0 &&
        state.invalidPartCount == 0 &&
        state.spacingPenalty <= 1e-9 &&
        state.sheetPenalty <= 1e-9;
}

inline LayoutState solveBenchmark(const std::filesystem::path& root, const wchar_t* fileName, EngineSettings settings, SolverStats& stats) {
    Document document = loadBenchmark(root, fileName, settings);
    std::atomic_bool stop{false};
    MultiStartSolver solver;
    LayoutState solved = solver.solve(document, settings, stop, {});
    PenaltySystem penalties;
    solved = LayoutScore{}.evaluate(document, settings, solved.poses, &penalties);
    stats = solver.lastStats();
    return solved;
}

inline EmptySpaceMap analyzeBenchmark(const std::filesystem::path& root, const wchar_t* fileName, const EngineSettings& settings, const LayoutState& state) {
    Document document = loadBenchmark(root, fileName, settings);
    return EmptySpaceAnalyzer{}.analyze(document, settings, state);
}

inline void printClusterStats(const char* caseName, const LayoutState& state, const SolverStats& stats, const EmptySpaceMap& map) {
    std::cout << caseName
              << " util=" << state.utilization
              << " coordinatedAttempts=" << stats.coordinatedClusterRebuildAttempts
              << " coordinatedAccepted=" << stats.coordinatedClusterRebuildAccepted
              << " denseAttempts=" << stats.denseSmallPartCompactionAttempts
              << " denseAccepted=" << stats.denseSmallPartCompactionAccepted
              << " clusterBeamGenerated=" << stats.clusterBeamStatesGenerated
              << " clusterBeamKept=" << stats.clusterBeamStatesKept
              << " clusterBeamLeaves=" << stats.clusterBeamLeaves
              << " clusterBeamAccepted=" << stats.clusterBeamAccepted
              << " denseClusterBeamAccepted=" << stats.denseClusterBeamAccepted
              << " nfpGenerated=" << stats.nfpCandidatesGenerated
              << " nfpValid=" << stats.nfpCandidatesValid
              << " nfpAccepted=" << stats.nfpCandidatesAccepted
              << " ifpGenerated=" << stats.ifpCandidatesGenerated
              << " ifpValid=" << stats.ifpCandidatesValid
              << " ifpAccepted=" << stats.ifpCandidatesAccepted
              << " nfpCacheHits=" << stats.nfpCacheHits
              << " nfpCacheMisses=" << stats.nfpCacheMisses
              << " analyticFallbackGenerated=" << stats.analyticFallbackCandidatesGenerated
              << " restoreFallback=" << stats.clusterBeamRestoreFallbackCount
              << " restoreFallbackRatio=" << stats.clusterBeamRestoreFallbackRatio
              << " avgClusterBeamDepth=" << stats.clusterBeamAverageDepth
              << " usedReduction=" << stats.bestRebuildUsedAreaReduction
              << " widthReduction=" << stats.bestRebuildUsedWidthReduction
              << " heightReduction=" << stats.bestRebuildUsedHeightReduction
              << " utilGain=" << stats.bestRebuildUtilizationGain
              << " largestGap=" << map.largestRegionArea
              << " largestGapReduction=" << stats.bestRebuildLargestEmptyRegionReduction
              << "\n";
}

} // namespace nest::test_smoke
