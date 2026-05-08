#include "tests/solver_smoke_common.h"

#include <filesystem>
#include <iostream>

int wmain(int argc, wchar_t** argv) {
    if (argc < 2) {
        std::wcerr << L"usage: partial_cluster_state_smoke <repo-root>\n";
        return 2;
    }

    const std::filesystem::path root = argv[1];
    nest::EngineSettings settings = nest::test_smoke::maximumSettings();
    nest::SolverStats stats;
    nest::LayoutState solved = nest::test_smoke::solveBenchmark(root, L"mixed_100_parts.svg", settings, stats);
    nest::EmptySpaceMap map = nest::test_smoke::analyzeBenchmark(root, L"mixed_100_parts.svg", settings, solved);
    nest::test_smoke::printClusterStats("partial_cluster_state mixed_100", solved, stats, map);

    const double restoreRatio = stats.clusterBeamStatesGenerated > 0
        ? static_cast<double>(stats.clusterBeamRestoreFallbackCount) / static_cast<double>(stats.clusterBeamStatesGenerated)
        : 1.0;

    bool ok = true;
    ok = nest::test_smoke::expect("strict validity", nest::test_smoke::valid(solved)) && ok;
    ok = nest::test_smoke::expect("partial beam generated alternatives", stats.clusterBeamStatesGenerated > 1 && stats.clusterBeamStatesKept > 1) && ok;
    ok = nest::test_smoke::expect("unplaced cluster shadow is not dominating rank", stats.clusterBeamAverageDepth >= 10.0) && ok;
    ok = nest::test_smoke::expect("restore fallback ratio is measured", stats.clusterBeamStatesGenerated > 0) && ok;
    ok = nest::test_smoke::expect("restore fallback ratio is controlled", restoreRatio <= 0.30) && ok;
    ok = nest::test_smoke::expect("beam evaluates full/meaningful cluster leaves", stats.clusterBeamLeaves > 0) && ok;
    ok = nest::test_smoke::expect("mixed_100 coordinated accepted target", stats.coordinatedClusterRebuildAccepted > 0) && ok;
    ok = nest::test_smoke::expect("mixed_100 cluster beam accepted target", stats.clusterBeamAccepted > 0) && ok;
    return ok ? 0 : 1;
}
