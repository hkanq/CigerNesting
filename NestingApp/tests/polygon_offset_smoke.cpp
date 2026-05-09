#include "geometry/polygon_offset.h"
#include "core/aabb.h"
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
    const nest::Ring square = rect(100.0, 80.0);
    const nest::Ring outward = nest::offsetRingConservative(square, {5.0, 1e-6, nest::OffsetJoinStyle::Miter, 4.0});
    const nest::Ring inward = nest::offsetRingConservative(square, {-5.0, 1e-6, nest::OffsetJoinStyle::Miter, 4.0});
    nest::AABB outBox;
    for (nest::Vec2 p : outward.points) outBox.include(p);
    nest::AABB inBox;
    for (nest::Vec2 p : inward.points) inBox.include(p);
    bool ok = true;
    ok = expect("outward offset expands square bounds", outBox.width() > 100.0 && outBox.height() > 80.0) && ok;
    ok = expect("inward offset shrinks square bounds", inBox.width() < 100.0 && inBox.height() < 80.0) && ok;
    ok = expect("offset preserves polygon topology", outward.points.size() == square.points.size() && inward.points.size() == square.points.size()) && ok;
    return ok ? 0 : 1;
}

