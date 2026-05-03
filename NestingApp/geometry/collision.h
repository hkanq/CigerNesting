#pragma once

#include "core/aabb.h"
#include "core/part.h"
#include "core/sheet.h"
#include "geometry/transformed_shape.h"
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

enum class PointLocation {
    Outside,
    Inside,
    OnBoundary
};

struct ClearanceSettings {
    double partSpacing = 0.0;
    double sheetMargin = 0.0;
    double tolerance = 1e-9;
};

bool segmentsIntersect(Vec2 a, Vec2 b, Vec2 c, Vec2 d, double eps = 1e-9);
PointLocation pointInRing(const std::vector<Vec2>& ring, Vec2 p, double eps = 1e-9);
PointLocation pointInRing(const Ring& ring, Vec2 p, double eps = 1e-9);
bool ringsOverlapOrTouch(const std::vector<Vec2>& a, const std::vector<Vec2>& b, double eps = 1e-9);
bool ringsOverlapOrTouch(const TransformedRing& a, const TransformedRing& b, double eps = 1e-9);
bool pointInSolidArea(const TransformedPart& part, Vec2 p, double eps = 1e-9);
bool partsCollide(const Part& a, const Pose& poseA, const Part& b, const Pose& poseB, double eps = 1e-9);
bool partsRespectSpacing(const Part& a, const Pose& poseA, const Part& b, const Pose& poseB, const ClearanceSettings& clearance);
bool partRespectsSheetMargin(const Part& part, const Pose& pose, const Sheet& sheet, const ClearanceSettings& clearance);
bool isPartInsideSheet(const Part& part, const Pose& pose, const Sheet& sheet, double eps);
bool overlapsSheetHolesOrForbiddenZones(const Part& part, const Pose& pose, const Sheet& sheet, double eps);
double minimumDistanceToSheetFeatures(const Part& part, const Pose& pose, const Sheet& sheet, double eps = 1e-9);

bool ringsOverlap(const Ring& a, const Ring& b, double tolerance = 1e-9);
bool partsOverlap(const Part& a, const Pose& poseA, const Part& b, const Pose& poseB, double tolerance = 1e-9);
bool isPartInsideSheet(const Part& part, const Pose& pose, const Sheet& sheet);
bool overlapsSheetHolesOrForbiddenZones(const Part& part, const Pose& pose, const Sheet& sheet);
double aabbOverlapArea(const AABB& a, const AABB& b);

} // namespace nest
