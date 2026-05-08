#include "tests/nfp_provider_test_common.h"

int main() {
    nest::Document document = nest::nfp_test::simpleDocument();
    nest::EngineSettings settings = nest::nfp_test::settings();
    std::vector<nest::Pose> poses(2);
    poses[0].x = 120.0;
    poses[0].y = 100.0;
    nest::ContactCandidateRequest request = nest::nfp_test::requestFor(1, {0});
    nest::NfpCandidateProvider provider;
    nest::ContactCandidateStats first;
    (void)provider.generatePartPartCandidates(document, settings, poses, request, &first);
    nest::ContactCandidateStats second;
    (void)provider.generatePartPartCandidates(document, settings, poses, request, &second);
    std::cout << "nfp cache firstMisses=" << first.cacheMisses << " firstHits=" << first.cacheHits << " secondMisses=" << second.cacheMisses << " secondHits=" << second.cacheHits << "\n";
    bool ok = true;
    ok = nest::nfp_test::expect("first call populates cache", first.cacheMisses > 0) && ok;
    ok = nest::nfp_test::expect("second call hits cache", second.cacheHits > 0) && ok;
    return ok ? 0 : 1;
}
