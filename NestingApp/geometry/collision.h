#pragma once

#include "core/aabb.h"
#include "core/part.h"
#include "core/sheet.h"
#include <cstddef>
#include <utility>
#include <vector>

namespace nest {

struct CollisionPair {
    size_t a = 0;
    size_t b = 0;
};

struct CollisionReport {
    size_t collisionCount = 0;
    double overlapScore = 0.0;
    std::vector<CollisionPair> pairs;
};

bool segmentsIntersect(Vec2 a, Vec2 b, Vec2 c, Vec2 d, double tolerance = 1e-9);
bool ringsOverlap(const Ring& a, const Ring& b, double tolerance = 1e-9);
bool partsOverlap(const Part& a, const Pose& poseA, const Part& b, const Pose& poseB, double tolerance = 1e-9);
bool isPartInsideSheet(const Part& part, const Pose& pose, const Sheet& sheet);
bool overlapsSheetHolesOrForbiddenZones(const Part& part, const Pose& pose, const Sheet& sheet);
double aabbOverlapArea(const AABB& a, const AABB& b);

} // namespace nest
