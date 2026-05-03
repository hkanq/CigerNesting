#include "engine/layout_score.h"

#include "core/aabb.h"
#include "engine/broadphase.h"
#include "geometry/collision.h"
#include <algorithm>

namespace nest {
namespace {

constexpr double kCollisionPenalty = 100000000.0;
constexpr double kSheetPenalty = 100000000.0;
constexpr double kSpacingPenalty = 1000000.0;
constexpr double kUsedAreaPenalty = 1.0;
constexpr double kUtilizationReward = 10000.0;
constexpr double kCompactnessReward = 0.02;

double safeSheetArea(const Document& document) {
    return std::max(1.0, (document.sheet.width - document.sheet.margin * 2.0) * (document.sheet.height - document.sheet.margin * 2.0));
}

} // namespace

LayoutState LayoutScore::evaluate(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const PenaltySystem* attemptPenalties,
    const PenaltySystem* globalPenalties,
    double globalPenaltyWeight) const {
    LayoutState state;
    state.poses = poses;

    if (document.parts.empty() || poses.empty()) {
        return state;
    }

    AABB used;
    const size_t count = std::min(document.parts.size(), poses.size());
    for (size_t i = 0; i < count; ++i) {
        used.include(transformedBounds(document.parts[i], poses[i]));
    }
    state.usedWidth = used.width();
    state.usedHeight = used.height();
    const double usedArea = std::max(1.0, used.area());
    state.utilization = std::max(0.0, std::min(1.0, document.totalPartArea() / usedArea));
    const double compactness = document.totalPartArea() / usedArea;

    BroadPhase broad;
    const auto pairs = broad.findCandidatePairs(document.parts, poses, settings.partSpacing);
    const ClearanceSettings clearance{settings.partSpacing, settings.margin, settings.collisionTolerance};

    for (const auto& [a, b] : pairs) {
        if (a >= document.parts.size() || b >= document.parts.size() || a >= poses.size() || b >= poses.size()) {
            continue;
        }
        const AABB boxA = transformedBounds(document.parts[a], poses[a]);
        const AABB boxB = transformedBounds(document.parts[b], poses[b]);
        const double attemptWeight = attemptPenalties ? attemptPenalties->weight(a, b) : 1.0;
        const double globalWeight = globalPenalties ? globalPenalties->weight(a, b) : 1.0;
        const double pairWeight = attemptWeight * (1.0 + std::max(0.0, globalPenaltyWeight) * std::max(0.0, globalWeight - 1.0));
        if (partsCollide(document.parts[a], poses[a], document.parts[b], poses[b], settings.collisionTolerance)) {
            ++state.collisionCount;
            state.collisionPairs.push_back({a, b});
            state.overlapPenalty += pairWeight * (1.0 + aabbOverlapArea(boxA, boxB));
        } else if (!partsRespectSpacing(document.parts[a], poses[a], document.parts[b], poses[b], clearance)) {
            state.spacingPenalty += pairWeight;
        }
    }

    for (size_t i = 0; i < count; ++i) {
        bool invalid = false;
        if (!isPartInsideSheet(document.parts[i], poses[i], document.sheet, settings.collisionTolerance) ||
            overlapsSheetHolesOrForbiddenZones(document.parts[i], poses[i], document.sheet, settings.collisionTolerance)) {
            invalid = true;
        }
        if (!partRespectsSheetMargin(document.parts[i], poses[i], document.sheet, clearance)) {
            invalid = true;
            state.sheetPenalty += 1.0;
        }
        if (invalid) {
            ++state.invalidPartCount;
            state.sheetPenalty += 1.0;
        }
    }

    state.totalScore =
        static_cast<double>(state.collisionCount) * kCollisionPenalty +
        state.overlapPenalty * kCollisionPenalty * 0.00001 +
        static_cast<double>(state.invalidPartCount) * kSheetPenalty +
        state.sheetPenalty * kSheetPenalty +
        state.spacingPenalty * kSpacingPenalty +
        usedArea * kUsedAreaPenalty -
        state.utilization * kUtilizationReward -
        compactness * kCompactnessReward;

    return state;
}

} // namespace nest
