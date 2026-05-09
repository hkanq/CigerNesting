#include "tests/nfp_provider_test_common.h"

int main() {
    nest::Document document = nest::nfp_test::simpleDocument();
    nest::EngineSettings settings = nest::nfp_test::settings();
    std::vector<nest::Pose> poses(2);
    poses[0].x = 120.0;
    poses[0].y = 100.0;
    nest::ContactCandidateRequest request = nest::nfp_test::requestFor(1, {0});
    nest::ContactCandidateStats stats;
    nest::InnerFitCandidateProvider provider;
    std::vector<nest::ContactCandidate> candidates = provider.generatePartSheetCandidates(document, settings, poses, request, &stats);
    std::cout << "ifp generated=" << stats.generated << " valid=" << stats.valid << " candidates=" << candidates.size() << "\n";
    bool ok = true;
    ok = nest::nfp_test::expect("IFP generated sheet candidates", stats.generated > 0 && !candidates.empty()) && ok;
    ok = nest::nfp_test::expect("real IFP loops generated", stats.ifpLoopsGenerated > 0) && ok;
    ok = nest::nfp_test::expect("IFP has valid candidates", stats.valid > 0) && ok;
    ok = nest::nfp_test::expect("IFP candidates are exact-valid", std::all_of(candidates.begin(), candidates.end(), [&](const nest::ContactCandidate& c) {
        return nest::nfp_test::validCandidate(document, settings, poses, 1, c.pose, {0});
    })) && ok;
    return ok ? 0 : 1;
}
