#include "geometry/winding.h"

namespace nest::tests {

bool geometryAreaSmokeTest() {
    return signedArea({{0, 0}, {10, 0}, {10, 10}, {0, 10}}) > 0.0;
}

} // namespace nest::tests
