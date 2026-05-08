#include "tests/solver_smoke_common.h"

#include <filesystem>
#include <iostream>

int wmain(int argc, wchar_t** argv) {
    if (argc < 2) {
        std::wcerr << L"usage: cluster_compaction_smoke <repo-root>\n";
        return 2;
    }

    const std::filesystem::path root = argv[1];
    nest::EngineSettings settings = nest::test_smoke::maximumSettings();
    nest::SolverStats stats;
    nest::LayoutState solved = nest::test_smoke::solveBenchmark(root, L"mixed_100_parts.svg", settings, stats);
    nest::EmptySpaceMap map = nest::test_smoke::analyzeBenchmark(root, L"mixed_100_parts.svg", settings, solved);
    nest::test_smoke::printClusterStats("cluster_compaction mixed_100", solved, stats, map);

    bool ok = true;
    ok = nest::test_smoke::expect("strict validity", nest::test_smoke::valid(solved)) && ok;
    ok = nest::test_smoke::expect("cluster rebuild attempted", stats.coordinatedClusterRebuildAttempts > 0) && ok;
    ok = nest::test_smoke::expect("cluster beam generated multiple states", stats.clusterBeamStatesGenerated > 1) && ok;
    ok = nest::test_smoke::expect("cluster beam kept alternatives", stats.clusterBeamStatesKept > 1) && ok;
    ok = nest::test_smoke::expect("cluster beam evaluated valid leaves", stats.clusterBeamLeaves > 0) && ok;
    ok = nest::test_smoke::expect("cluster beam accepted a quality move", stats.clusterBeamAccepted > 0) && ok;
    ok = nest::test_smoke::expect("coordinated cluster accepted", stats.coordinatedClusterRebuildAccepted > 0) && ok;
    ok = nest::test_smoke::expect("used area or utilization improved",
        stats.bestRebuildUsedAreaReduction > 1.0 ||
        stats.bestRebuildUsedWidthReduction > 0.25 ||
        stats.bestRebuildUsedHeightReduction > 0.25 ||
        stats.bestRebuildUtilizationGain > 0.001) && ok;
    return ok ? 0 : 1;
}
