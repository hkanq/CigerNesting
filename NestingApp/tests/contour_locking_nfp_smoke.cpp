#include "tests/nfp_provider_test_common.h"

int main() {
    nest::Document document;
    document.sheet.width = 360.0;
    document.sheet.height = 260.0;
    document.addPart(nest::nfp_test::cShapePart());
    document.addPart(nest::nfp_test::squarePart(L"filler", 30.0, 30.0));
    nest::EngineSettings settings = nest::nfp_test::settings();
    std::vector<nest::Pose> poses(2);
    poses[0].x = 100.0;
    poses[0].y = 70.0;
    nest::ContactCandidateRequest request = nest::nfp_test::requestFor(1, {0});
    request.candidateLimit = 128;
    nest::NfpCandidateProvider provider;
    nest::ContactCandidateStats stats;
    std::vector<nest::ContactCandidate> candidates = provider.generatePartPartCandidates(document, settings, poses, request, &stats);
    const bool hasLock = std::any_of(candidates.begin(), candidates.end(), [](const nest::ContactCandidate& c) {
        return c.kind == nest::AnalyticContactKind::NfpPartPart || c.kind == nest::AnalyticContactKind::EdgeParallel;
    });
    std::cout << "contour locking nfp generated=" << stats.generated << " valid=" << stats.valid << " candidates=" << candidates.size() << "\n";
    bool ok = true;
    ok = nest::nfp_test::expect("contour-locking NFP candidates generated", stats.generated > 0 && !candidates.empty()) && ok;
    ok = nest::nfp_test::expect("contour-locking candidate kind present", hasLock) && ok;
    ok = nest::nfp_test::expect("contour-locking candidates are exact-valid", std::all_of(candidates.begin(), candidates.end(), [&](const nest::ContactCandidate& c) {
        return nest::nfp_test::validCandidate(document, settings, poses, 1, c.pose, {0});
    })) && ok;
    return ok ? 0 : 1;
}
