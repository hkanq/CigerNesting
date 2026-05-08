#include "tests/solver_smoke_common.h"

#include <filesystem>
#include <iostream>

int wmain(int argc, wchar_t** argv) {
    if (argc < 2) {
        std::wcerr << L"usage: nfp_constructive_rebuild_smoke <repo-root>\n";
        return 2;
    }
    const std::filesystem::path root = argv[1];
    nest::EngineSettings settings = nest::test_smoke::maximumSettings();
    nest::SolverStats stats;
    nest::LayoutState solved = nest::test_smoke::solveBenchmark(root, L"mixed_100_parts.svg", settings, stats);
    nest::EmptySpaceMap map = nest::test_smoke::analyzeBenchmark(root, L"mixed_100_parts.svg", settings, solved);
    nest::test_smoke::printClusterStats("nfp_constructive mixed_100", solved, stats, map);
    bool ok = true;
    ok = nest::test_smoke::expect("strict validity", nest::test_smoke::valid(solved)) && ok;
    ok = nest::test_smoke::expect("NFP provider generated candidates", stats.nfpCandidatesGenerated > 0) && ok;
    ok = nest::test_smoke::expect("NFP provider produced exact-valid candidates", stats.nfpCandidatesValid > 0) && ok;
    ok = nest::test_smoke::expect("NFP candidates entered cluster beam", stats.nfpCandidatesAccepted > 0) && ok;
    ok = nest::test_smoke::expect("IFP candidates entered cluster beam", stats.ifpCandidatesAccepted > 0) && ok;
    ok = nest::test_smoke::expect("NFP cache is active", stats.nfpCacheHits > 0 && stats.nfpCacheMisses > 0) && ok;
    ok = nest::test_smoke::expect("cluster beam accepted target", stats.clusterBeamAccepted > 0) && ok;
    ok = nest::test_smoke::expect("mixed_100 utilization short target", solved.utilization >= 0.65) && ok;
    return ok ? 0 : 1;
}
