#include "tests/nfp_provider_test_common.h"

int main() {
    nest::Document document = nest::nfp_test::simpleDocument();
    nest::EngineSettings settings = nest::nfp_test::settings();
    std::vector<nest::Pose> poses(2);
    poses[0].x = 120.0;
    poses[0].y = 100.0;
    nest::ContactCandidateRequest request = nest::nfp_test::requestFor(1, {0});
    nest::ContactCandidateStats stats;
    nest::NfpCandidateProvider provider;
    std::vector<nest::ContactCandidate> candidates = provider.generatePartPartCandidates(document, settings, poses, request, &stats);
    std::cout << "nfp simple generated=" << stats.generated << " valid=" << stats.valid << " cacheHits=" << stats.cacheHits << " cacheMisses=" << stats.cacheMisses << " candidates=" << candidates.size() << "\n";
    bool ok = true;
    ok = nest::nfp_test::expect("NFP generated candidates", stats.generated > 0 && !candidates.empty()) && ok;
    ok = nest::nfp_test::expect("NFP has valid candidates", stats.valid > 0) && ok;
    ok = nest::nfp_test::expect("NFP candidates are exact-valid", std::all_of(candidates.begin(), candidates.end(), [&](const nest::ContactCandidate& c) {
        return nest::nfp_test::validCandidate(document, settings, poses, 1, c.pose, {0});
    })) && ok;
    return ok ? 0 : 1;
}
