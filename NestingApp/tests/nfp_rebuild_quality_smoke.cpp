#include "tests/solver_smoke_common.h"

#include <filesystem>
#include <iostream>

int wmain(int argc, wchar_t** argv) {
    if (argc < 2) {
        std::wcerr << L"usage: nfp_rebuild_quality_smoke <repo-root>\n";
        return 2;
    }
    const std::filesystem::path root = argv[1];
    nest::EngineSettings settings = nest::test_smoke::maximumSettings();
    nest::SolverStats stats;
    nest::LayoutState solved = nest::test_smoke::solveBenchmark(root, L"many_small_parts.svg", settings, stats);
    nest::EmptySpaceMap map = nest::test_smoke::analyzeBenchmark(root, L"many_small_parts.svg", settings, solved);
    nest::test_smoke::printClusterStats("nfp_rebuild many_small", solved, stats, map);
    bool ok = true;
    ok = nest::test_smoke::expect("strict validity", nest::test_smoke::valid(solved)) && ok;
    ok = nest::test_smoke::expect("real NFP loops generated", stats.nfpLoopsGenerated > 0) && ok;
    ok = nest::test_smoke::expect("real NFP loop candidates generated", stats.nfpLoopCandidatesGenerated > 0) && ok;
    ok = nest::test_smoke::expect("NFP exact-valid candidates", stats.nfpCandidatesValid > 0) && ok;
    ok = nest::test_smoke::expect("many_small short utilization target", solved.utilization > 0.62) && ok;
    return ok ? 0 : 1;
}
