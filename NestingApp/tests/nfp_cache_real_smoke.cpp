#include "tests/nfp_provider_test_common.h"
#include "engine/nfp_solver_cache.h"
#include "geometry/no_fit_polygon.h"
#include <iostream>

int main() {
    nest::Document document = nest::nfp_test::simpleDocument();
    nest::Pose moving;
    nest::Pose fixed;
    nest::NoFitPolygonOptions options;
    const nest::NoFitPolygonResult nfp = nest::buildNoFitPolygon(document.parts[1], moving, document.parts[0], fixed, options);
    nest::NfpSolverCache cache;
    nest::NfpSolverCacheKey key;
    key.movingPartId = 1;
    key.fixedPartId = 0;
    key.geometryVersion = nest::nfpSolverGeometryVersion(document.parts[1], document.parts[0], 1, 0);
    key.toleranceBucket = nest::nfpSolverToleranceBucket(options.tolerance);
    cache.store(key, nest::toSolverCacheValue(nfp));
    nest::NfpSolverCacheValue value;
    const bool hit = cache.find(key, value);
    bool ok = true;
    ok = nest::nfp_test::expect("real NFP solver cache hit", hit && cache.hitCount() == 1) && ok;
    ok = nest::nfp_test::expect("cache stores NFP loops, not only anchor candidates", !value.loops.empty() && value.componentCount == nfp.componentCount) && ok;
    return ok ? 0 : 1;
}
