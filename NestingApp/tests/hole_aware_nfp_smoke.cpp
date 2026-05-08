#include "tests/nfp_provider_test_common.h"

int main() {
    nest::Document document;
    document.sheet.width = 300.0;
    document.sheet.height = 300.0;
    document.addPart(nest::nfp_test::donutPart());
    document.addPart(nest::nfp_test::squarePart(L"small", 20.0, 20.0));
    nest::EngineSettings settings = nest::nfp_test::settings();
    std::vector<nest::Pose> poses(2);
    poses[0].x = 80.0;
    poses[0].y = 80.0;
    nest::ContactCandidateRequest request = nest::nfp_test::requestFor(1, {0});
    request.candidateLimit = 128;
    nest::NfpCandidateProvider provider;
    nest::ContactCandidateStats stats;
    std::vector<nest::ContactCandidate> candidates = provider.generatePartHoleCandidates(document, settings, poses, request, &stats);
    std::cout << "hole-aware nfp generated=" << stats.generated << " valid=" << stats.valid << " candidates=" << candidates.size() << "\n";
    bool ok = true;
    ok = nest::nfp_test::expect("hole-aware NFP candidates generated", stats.generated > 0) && ok;
    ok = nest::nfp_test::expect("hole-aware NFP valid candidates generated", !candidates.empty()) && ok;
    ok = nest::nfp_test::expect("hole candidates are exact-valid", std::all_of(candidates.begin(), candidates.end(), [&](const nest::ContactCandidate& c) {
        return nest::nfp_test::validCandidate(document, settings, poses, 1, c.pose, {0});
    })) && ok;
    return ok ? 0 : 1;
}
