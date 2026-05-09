#include "geometry/minkowski_sum.h"
#include <iostream>

namespace {

nest::Ring rect(double w, double h) {
    nest::Ring ring;
    ring.points = {{0.0, 0.0}, {w, 0.0}, {w, h}, {0.0, h}};
    return ring;
}

bool expect(const char* name, bool condition) {
    std::cout << (condition ? "PASS: " : "QUALITY_FAIL: ") << name << "\n";
    return condition;
}

} // namespace

int main() {
    bool ok = true;
    const nest::Ring a = rect(80.0, 60.0);
    const nest::Ring b = rect(40.0, 30.0);
    const nest::MinkowskiResult convex = nest::minkowskiSumDecomposed(a, b, 1e-6);
    ok = expect("convex rectangles produce one component", convex.components.size() == 1) && ok;
    if (!convex.components.empty()) {
        ok = expect("rectangle sum has four or more boundary points", convex.components.front().ring.points.size() >= 4) && ok;
        ok = expect("convex path marked exact", convex.components.front().exactConvex) && ok;
    }

    nest::Ring lShape;
    lShape.points = {{0.0, 0.0}, {80.0, 0.0}, {80.0, 25.0}, {30.0, 25.0}, {30.0, 75.0}, {0.0, 75.0}};
    const nest::MinkowskiResult concave = nest::minkowskiSumDecomposed(lShape, b, 1e-6);
    ok = expect("concave polygon decomposes into multiple Minkowski components", concave.usedDecomposition && concave.components.size() > 1) && ok;
    return ok ? 0 : 1;
}
