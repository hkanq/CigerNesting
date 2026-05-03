#include "geometry/collision.h"

namespace nest::tests {

bool segmentIntersectionSmokeTest() {
    return segmentsIntersect({0, 0}, {10, 10}, {0, 10}, {10, 0});
}

} // namespace nest::tests
