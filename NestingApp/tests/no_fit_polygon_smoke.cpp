#include "tests/nfp_provider_test_common.h"
#include "geometry/no_fit_polygon.h"
#include <iostream>

int main() {
    nest::Document document = nest::nfp_test::simpleDocument();
    nest::Pose moving;
    nest::Pose fixed;
    nest::NoFitPolygonOptions options;
    options.spacing = 0.0;
    options.tolerance = 1e-6;
    const nest::NoFitPolygonResult nfp = nest::buildNoFitPolygon(document.parts[1], moving, document.parts[0], fixed, options);
    bool ok = true;
    ok = nest::nfp_test::expect("real NFP builds boundary loops", !nfp.loops.empty()) && ok;
    ok = nest::nfp_test::expect("NFP component count matches loops", nfp.componentCount == nfp.loops.size()) && ok;
    if (!nfp.loops.empty()) {
        const nest::AABB box = nfp.loops.front().bounds;
        ok = nest::nfp_test::expect("rectangle NFP min translation includes moving negative extent", box.min.x <= -39.9 && box.min.y <= -29.9) && ok;
        ok = nest::nfp_test::expect("rectangle NFP max translation includes fixed extent", box.max.x >= 79.9 && box.max.y >= 59.9) && ok;
    }
    const std::vector<nest::Pose> poses = nest::sampleNoFitPolygonCandidates(nfp, moving, 32, {0.0, 0.0});
    ok = nest::nfp_test::expect("NFP loop samples candidate poses", !poses.empty()) && ok;
    return ok ? 0 : 1;
}
