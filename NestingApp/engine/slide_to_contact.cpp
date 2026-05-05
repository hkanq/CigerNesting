#include "engine/slide_to_contact.h"

#include "geometry/clearance.h"
#include "geometry/collision.h"
#include "geometry/transformed_shape.h"
#include <algorithm>

namespace nest {
namespace {

bool poseValidForLayout(
    const Document& document,
    const EngineSettings& settings,
    const LayoutState& state,
    size_t partIndex,
    const Pose& pose) {
    if (partIndex >= document.parts.size()) {
        return false;
    }
    const Part& part = document.parts[partIndex];
    if (!isPartInsideSheet(part, pose, document.sheet, settings.collisionTolerance) ||
        !partRespectsSheetClearance(part, pose, document.sheet, settings.margin, settings.collisionTolerance)) {
        return false;
    }
    const AABB bounds = transformedBounds(part, pose).expanded(settings.partSpacing + settings.collisionTolerance);
    for (size_t i = 0; i < document.parts.size() && i < state.poses.size(); ++i) {
        if (i == partIndex) {
            continue;
        }
        const AABB otherBounds = transformedBounds(document.parts[i], state.poses[i]);
        if (!bounds.overlaps(otherBounds, settings.collisionTolerance)) {
            continue;
        }
        if (partsCollide(part, pose, document.parts[i], state.poses[i], settings.collisionTolerance) ||
            !partsRespectClearance(part, pose, document.parts[i], state.poses[i], settings.partSpacing, settings.collisionTolerance)) {
            return false;
        }
    }
    return true;
}

Pose translated(Pose pose, Vec2 direction, double distance) {
    pose.x += direction.x * distance;
    pose.y += direction.y * distance;
    return pose;
}

} // namespace

SlideToContactResult SlideToContact::slide(
    const Document& document,
    const EngineSettings& settings,
    const LayoutState& state,
    size_t partIndex,
    const Pose& initialPose,
    Vec2 direction,
    double maxDistance) const {
    SlideToContactResult result;
    result.pose = initialPose;
    const double len = direction.length();
    if (len <= 1e-9 || maxDistance <= 1e-9 || partIndex >= state.poses.size()) {
        return result;
    }
    direction.x /= len;
    direction.y /= len;

    if (!poseValidForLayout(document, settings, state, partIndex, initialPose)) {
        return result;
    }

    double low = 0.0;
    double high = std::max(0.0, maxDistance);
    double probe = std::min(high, std::max(1.0, high / 8.0));
    while (probe < high && poseValidForLayout(document, settings, state, partIndex, translated(initialPose, direction, probe))) {
        low = probe;
        probe = std::min(high, probe * 2.0);
    }
    if (poseValidForLayout(document, settings, state, partIndex, translated(initialPose, direction, probe))) {
        low = probe;
    } else {
        high = probe;
    }

    for (int i = 0; i < 24; ++i) {
        const double mid = (low + high) * 0.5;
        if (poseValidForLayout(document, settings, state, partIndex, translated(initialPose, direction, mid))) {
            low = mid;
        } else {
            high = mid;
        }
    }

    if (low > std::max(0.01, settings.collisionTolerance)) {
        result.pose = translated(initialPose, direction, low);
        result.moved = true;
        result.distance = low;
    }
    return result;
}

} // namespace nest
