#include "tests/nfp_provider_test_common.h"
#include "geometry/inner_fit_polygon.h"
#include <cmath>
#include <iostream>

int main() {
    nest::Document document = nest::nfp_test::simpleDocument();
    nest::Pose orientation;
    nest::InnerFitPolygonOptions options;
    options.margin = 0.0;
    options.tolerance = 1e-6;
    const nest::InnerFitPolygonResult ifp = nest::buildInnerFitPolygon(document.parts[1], orientation, document.sheet, options);
    bool ok = true;
    ok = nest::nfp_test::expect("rectangular IFP produces exact region", ifp.exactRectangular && !ifp.loops.empty()) && ok;
    if (!ifp.loops.empty()) {
        const nest::AABB box = ifp.loops.front().bounds;
        ok = nest::nfp_test::expect("IFP left/bottom at zero", std::abs(box.min.x) <= 1e-6 && std::abs(box.min.y) <= 1e-6) && ok;
        ok = nest::nfp_test::expect("IFP right/top respect part bounds", box.max.x >= 359.9 && box.max.y >= 269.9) && ok;
    }
    const std::vector<nest::Pose> poses = nest::sampleInnerFitPolygonCandidates(ifp, orientation, 16, {200.0, 150.0});
    ok = nest::nfp_test::expect("IFP samples candidate poses", !poses.empty()) && ok;
    return ok ? 0 : 1;
}

