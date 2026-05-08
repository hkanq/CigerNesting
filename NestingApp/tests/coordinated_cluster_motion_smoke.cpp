#include "tests/solver_smoke_common.h"

#include <filesystem>
#include <iostream>

int wmain(int argc, wchar_t** argv) {
    if (argc < 2) {
        std::wcerr << L"usage: coordinated_cluster_motion_smoke <repo-root>\n";
        return 2;
    }

    const std::filesystem::path root = argv[1];
    nest::EngineSettings settings = nest::test_smoke::maximumSettings();
    nest::SolverStats stats;
    nest::LayoutState solved = nest::test_smoke::solveBenchmark(root, L"mixed_100_parts.svg", settings, stats);
    nest::EmptySpaceMap map = nest::test_smoke::analyzeBenchmark(root, L"mixed_100_parts.svg", settings, solved);
    nest::test_smoke::printClusterStats("coordinated_cluster_motion mixed_100", solved, stats, map);

    bool ok = true;
    ok = nest::test_smoke::expect("strict validity", nest::test_smoke::valid(solved)) && ok;
    ok = nest::test_smoke::expect("beam state tree is not single path", stats.clusterBeamStatesGenerated > stats.clusterBeamLeaves) && ok;
    ok = nest::test_smoke::expect("beam depth is meaningful", stats.clusterBeamAverageDepth >= 4.0) && ok;
    ok = nest::test_smoke::expect("coordinated rebuild or push accepted",
        stats.coordinatedClusterRebuildAccepted > 0 || stats.coordinatedClusterMotionAccepted > 0) && ok;
    ok = nest::test_smoke::expect("cluster beam accepted", stats.clusterBeamAccepted > 0) && ok;
    ok = nest::test_smoke::expect("restore fallback is not the dominant result",
        stats.clusterBeamRestoreFallbackCount < stats.clusterBeamStatesGenerated) && ok;
    return ok ? 0 : 1;
}
