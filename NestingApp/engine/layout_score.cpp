#include "engine/layout_score.h"

#include "core/aabb.h"
#include "engine/broadphase.h"
#include "engine/layout_score_components.h"
#include "geometry/clearance.h"
#include "geometry/collision.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace nest {
namespace {

constexpr double kCollisionPenalty = 100000000.0;
constexpr double kSheetPenalty = 100000000.0;
constexpr double kSpacingPenalty = 1000000.0;
constexpr double kUsedAreaPenalty = 2.40;
constexpr double kUtilizationReward = 60000.0;
constexpr double kCompactnessReward = 0.02;
constexpr double kCavityReward = 500000.0;
constexpr double kContactReward = 450.0;

double safeSheetArea(const Document& document) {
    return std::max(1.0, (document.sheet.width - document.sheet.margin * 2.0) * (document.sheet.height - document.sheet.margin * 2.0));
}

Ring physicalSheetOuter(const Sheet& sheet) {
    if (sheet.hasCustomProfile() && !sheet.profile().outerContour.points.empty()) {
        return sheet.profile().outerContour;
    }
    return sheet.makeRectangularOuterContour();
}

double sheetFeatureDistance(const Part& part, const Pose& pose, const Sheet& sheet, double requiredMargin, double eps) {
    const TransformedPart transformed = transformPart(part, pose);
    double best = minimumPartToRingDistance(transformed, physicalSheetOuter(sheet), requiredMargin, eps).minDistance;
    for (const Ring& hole : sheet.profile().holes) {
        best = std::min(best, minimumPartToRingDistance(transformed, hole, requiredMargin, eps).minDistance);
    }
    for (const Ring& zone : sheet.profile().forbiddenZones) {
        best = std::min(best, minimumPartToRingDistance(transformed, zone, requiredMargin, eps).minDistance);
    }
    return best;
}

double clearanceDeficit(double required, double actual) {
    if (!std::isfinite(actual)) {
        return 0.0;
    }
    return std::max(0.0, required - actual) / std::max(1.0, required);
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
    const double cavityReward = cavityPlacementReward(document, poses);
    const LayoutShapeMetrics shapeMetrics = computeLayoutShapeMetrics(document, settings, used);
    const bool qualityProfile = settings.performanceProfile != PerformanceProfile::Fast;
    const double totalPartArea = std::max(1.0, document.totalPartArea());
    const double usedAreaWeight = qualityProfile ? 1.85 : kUsedAreaPenalty;
    const double utilizationReward = settings.performanceProfile == PerformanceProfile::Maximum
        ? 115000.0
        : (qualityProfile ? 85000.0 : kUtilizationReward);
    const double cavityWeight = settings.performanceProfile == PerformanceProfile::Maximum
        ? 900000.0
        : (qualityProfile ? 700000.0 : kCavityReward);
    const double contactWeight = settings.performanceProfile == PerformanceProfile::Maximum
        ? 1050.0
        : (qualityProfile ? 760.0 : kContactReward);
    const double towerWeight = settings.performanceProfile == PerformanceProfile::Maximum
        ? 4.2
        : (qualityProfile ? 2.8 : 1.80);
    const double spreadPenalty = qualityProfile
        ? shapeMetrics.unusedSheetRegionScore * totalPartArea * 0.45
        : 0.0;
    const double towerPenalty = shapeMetrics.towerScore * totalPartArea * towerWeight;

    BroadPhase broad;
    const double contactSearchDistance = settings.performanceProfile == PerformanceProfile::Fast
        ? settings.partSpacing
        : std::max(settings.partSpacing, 6.0);
    const auto pairs = broad.findCandidatePairs(document.parts, poses, contactSearchDistance);
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
        } else {
            const ClearanceResult clearanceResult = minimumBoundaryDistance(
                transformPart(document.parts[a], poses[a], static_cast<int>(a)),
                transformPart(document.parts[b], poses[b], static_cast<int>(b)),
                clearance.partSpacing,
                settings.collisionTolerance);
            if (!clearanceResult.valid) {
                state.spacingPenalty += pairWeight * std::max(0.01, clearanceDeficit(clearance.partSpacing, clearanceResult.minDistance));
            } else {
                const double contactWindow = std::max(0.05, settings.collisionTolerance * 10.0);
                if (std::abs(clearanceResult.minDistance - clearance.partSpacing) <= contactWindow) {
                    state.contactReward += pairWeight;
                }
            }
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
            const double minDistance = sheetFeatureDistance(document.parts[i], poses[i], document.sheet, clearance.sheetMargin, settings.collisionTolerance);
            state.sheetPenalty += std::max(0.01, clearanceDeficit(clearance.sheetMargin, minDistance));
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
        usedArea * usedAreaWeight -
        state.utilization * utilizationReward -
        compactness * kCompactnessReward -
        cavityReward * cavityWeight -
        state.contactReward * contactWeight +
        towerPenalty +
        spreadPenalty;

    return state;
}

} // namespace nest
