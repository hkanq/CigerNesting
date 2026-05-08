#include "tests/solver_smoke_common.h"

#include <filesystem>
#include <iostream>

int wmain(int argc, wchar_t** argv) {
    if (argc < 2) {
        std::wcerr << L"usage: dense_nfp_packing_smoke <repo-root>\n";
        return 2;
    }
    const std::filesystem::path root = argv[1];
    nest::EngineSettings settings = nest::test_smoke::maximumSettings();
    nest::SolverStats stats;
    nest::LayoutState solved = nest::test_smoke::solveBenchmark(root, L"many_small_parts.svg", settings, stats);
    nest::EmptySpaceMap map = nest::test_smoke::analyzeBenchmark(root, L"many_small_parts.svg", settings, solved);
    nest::test_smoke::printClusterStats("dense_nfp many_small", solved, stats, map);
    bool ok = true;
    ok = nest::test_smoke::expect("strict validity", nest::test_smoke::valid(solved)) && ok;
    ok = nest::test_smoke::expect("dense NFP generated candidates", stats.nfpCandidatesGenerated > 0) && ok;
    ok = nest::test_smoke::expect("dense NFP candidates entered beam", stats.nfpCandidatesAccepted > 0) && ok;
    ok = nest::test_smoke::expect("dense cluster beam accepted target", stats.denseClusterBeamAccepted > 0) && ok;
    ok = nest::test_smoke::expect("many_small utilization target", solved.utilization >= 0.63) && ok;
    return ok ? 0 : 1;
}
