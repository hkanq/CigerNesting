#include "tests/solver_smoke_common.h"

#include <filesystem>
#include <iostream>

int wmain(int argc, wchar_t** argv) {
    if (argc < 2) {
        std::wcerr << L"usage: dense_small_part_compaction_smoke <repo-root>\n";
        return 2;
    }

    const std::filesystem::path root = argv[1];
    nest::EngineSettings settings = nest::test_smoke::maximumSettings();
    nest::SolverStats stats;
    nest::LayoutState solved = nest::test_smoke::solveBenchmark(root, L"many_small_parts.svg", settings, stats);
    nest::EmptySpaceMap map = nest::test_smoke::analyzeBenchmark(root, L"many_small_parts.svg", settings, solved);
    nest::test_smoke::printClusterStats("dense_small_part many_small", solved, stats, map);

    bool ok = true;
    ok = nest::test_smoke::expect("strict validity", nest::test_smoke::valid(solved)) && ok;
    ok = nest::test_smoke::expect("dense attempts are meaningful", stats.denseSmallPartCompactionAttempts >= 10) && ok;
    ok = nest::test_smoke::expect("coordinated attempts are meaningful", stats.coordinatedClusterRebuildAttempts >= 10) && ok;
    ok = nest::test_smoke::expect("dense beam accepted", stats.denseClusterBeamAccepted > 0) && ok;
    ok = nest::test_smoke::expect("dense accepted", stats.denseSmallPartCompactionAccepted > 0) && ok;
    ok = nest::test_smoke::expect("dense used-area signal exists",
        stats.bestRebuildUsedAreaReduction > 1.0 ||
        stats.bestRebuildUsedWidthReduction > 0.25 ||
        stats.bestRebuildUsedHeightReduction > 0.25 ||
        stats.bestRebuildUtilizationGain > 0.001) && ok;
    return ok ? 0 : 1;
}
