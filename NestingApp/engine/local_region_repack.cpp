#include "engine/local_region_repack.h"

#include "engine/empty_space_map.h"
#include "engine/layout_score.h"
#include "engine/layout_score_components.h"
#include "engine/pose_sampler.h"
#include "geometry/clearance.h"
#include "geometry/collision.h"
#include "geometry/transformed_shape.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

namespace nest {
namespace {

struct RepackSubset {
    std::vector<size_t> parts;
    double priority = 0.0;
};

struct PlacementCandidate {
    Pose pose;
    double localScore = std::numeric_limits<double>::max();
    double contact = 0.0;
};

struct BeamNode {
    std::vector<Pose> poses;
    std::vector<unsigned char> placed;
    size_t depth = 0;
    double score = std::numeric_limits<double>::max();
    double contact = 0.0;
};

enum class CandidateRejectReason {
    None,
    Sheet,
    Collision,
    Clearance
};

struct CandidateDiagnostics {
    size_t generated = 0;
    size_t valid = 0;
    size_t noCandidate = 0;
    size_t collisionReject = 0;
    size_t clearanceReject = 0;
    size_t sheetReject = 0;
    size_t scoreReject = 0;
    size_t beamPruned = 0;
    size_t fullValidationReject = 0;
    size_t maxCandidatesForPart = 0;
};

double footprint(const Part& part) {
    return part.area > 0.0 ? part.area : part.localBounds.area();
}

bool containsPart(const std::vector<size_t>& parts, size_t index) {
    return std::find(parts.begin(), parts.end(), index) != parts.end();
}

void appendUnique(std::vector<size_t>& parts, size_t index, size_t limit) {
    if (parts.size() >= limit || containsPart(parts, index)) {
        return;
    }
    parts.push_back(index);
}

void countReject(CandidateDiagnostics* diagnostics, CandidateRejectReason reason) {
    if (!diagnostics) {
        return;
    }
    switch (reason) {
    case CandidateRejectReason::Sheet:
        ++diagnostics->sheetReject;
        break;
    case CandidateRejectReason::Collision:
        ++diagnostics->collisionReject;
        break;
    case CandidateRejectReason::Clearance:
        ++diagnostics->clearanceReject;
        break;
    case CandidateRejectReason::None:
    default:
        break;
    }
}

double overlapArea(const AABB& a, const AABB& b) {
    if (!a.isValid() || !b.isValid()) {
        return 0.0;
    }
    const double minX = std::max(a.min.x, b.min.x);
    const double minY = std::max(a.min.y, b.min.y);
    const double maxX = std::min(a.max.x, b.max.x);
    const double maxY = std::min(a.max.y, b.max.y);
    if (minX >= maxX || minY >= maxY) {
        return 0.0;
    }
    return (maxX - minX) * (maxY - minY);
}

bool containsBounds(const AABB& outer, const AABB& inner, double tolerance) {
    return outer.isValid() && inner.isValid() &&
        inner.min.x + tolerance >= outer.min.x &&
        inner.min.y + tolerance >= outer.min.y &&
        inner.max.x <= outer.max.x + tolerance &&
        inner.max.y <= outer.max.y + tolerance;
}

double radians(double degrees) {
    constexpr double pi = 3.141592653589793238462643383279502884;
    return degrees * pi / 180.0;
}

Pose centerPose(const Part& part, double angle, bool mirrored, Vec2 center) {
    Pose orientation;
    orientation.angleRadians = angle;
    orientation.mirrored = mirrored;
    const AABB bounds = transformedBounds(part, orientation);
    orientation.x = center.x - bounds.center().x;
    orientation.y = center.y - bounds.center().y;
    return orientation;
}

std::vector<Vec2> sampleRing(const TransformedRing& ring, size_t limit) {
    std::vector<Vec2> points;
    if (ring.points.empty() || limit == 0) {
        return points;
    }
    const size_t count = ring.points.size();
    const size_t stride = std::max<size_t>(1, count / limit);
    for (size_t i = 0; i < count && points.size() < limit; i += stride) {
        points.push_back(ring.points[i]);
    }
    return points;
}

std::vector<Vec2> samplePartBoundary(const TransformedPart& part, size_t limitPerRing) {
    std::vector<Vec2> points;
    for (const TransformedRing& ring : part.rings) {
        std::vector<Vec2> sampled = sampleRing(ring, ring.isHole ? limitPerRing + 2u : limitPerRing);
        points.insert(points.end(), sampled.begin(), sampled.end());
    }
    return points;
}

bool validateAgainstPlacedDetailed(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const std::vector<unsigned char>& placed,
    size_t partIndex,
    const Pose& pose,
    CandidateRejectReason* rejectReason) {
    if (rejectReason) {
        *rejectReason = CandidateRejectReason::None;
    }
    if (partIndex >= document.parts.size()) {
        if (rejectReason) {
            *rejectReason = CandidateRejectReason::Sheet;
        }
        return false;
    }
    const Part& part = document.parts[partIndex];
    if (!isPartInsideSheet(part, pose, document.sheet, settings.collisionTolerance) ||
        !partRespectsSheetClearance(part, pose, document.sheet, settings.margin, settings.collisionTolerance)) {
        if (rejectReason) {
            *rejectReason = CandidateRejectReason::Sheet;
        }
        return false;
    }
    const AABB bounds = transformedBounds(part, pose).expanded(settings.partSpacing + settings.collisionTolerance);
    for (size_t i = 0; i < placed.size() && i < document.parts.size() && i < poses.size(); ++i) {
        if (!placed[i] || i == partIndex) {
            continue;
        }
        const AABB otherBounds = transformedBounds(document.parts[i], poses[i]);
        if (!bounds.overlaps(otherBounds, settings.collisionTolerance)) {
            continue;
        }
        if (partsCollide(part, pose, document.parts[i], poses[i], settings.collisionTolerance)) {
            if (rejectReason) {
                *rejectReason = CandidateRejectReason::Collision;
            }
            return false;
        }
        if (!partsRespectClearance(part, pose, document.parts[i], poses[i], settings.partSpacing, settings.collisionTolerance)) {
            if (rejectReason) {
                *rejectReason = CandidateRejectReason::Clearance;
            }
            return false;
        }
    }
    return true;
}

bool validAgainstPlaced(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const std::vector<unsigned char>& placed,
    size_t partIndex,
    const Pose& pose) {
    return validateAgainstPlacedDetailed(document, settings, poses, placed, partIndex, pose, nullptr);
}

Pose translated(Pose pose, Vec2 direction, double distance) {
    pose.x += direction.x * distance;
    pose.y += direction.y * distance;
    return pose;
}

Pose slideAgainstPlaced(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const std::vector<unsigned char>& placed,
    size_t partIndex,
    Pose pose,
    Vec2 direction,
    double maxDistance) {
    const double len = direction.length();
    if (len <= 1e-9 || maxDistance <= 1e-9 ||
        !validAgainstPlaced(document, settings, poses, placed, partIndex, pose)) {
        return pose;
    }
    direction.x /= len;
    direction.y /= len;
    double low = 0.0;
    double high = std::max(0.0, maxDistance);
    for (int i = 0; i < 22; ++i) {
        const double mid = (low + high) * 0.5;
        if (validAgainstPlaced(document, settings, poses, placed, partIndex, translated(pose, direction, mid))) {
            low = mid;
        } else {
            high = mid;
        }
    }
    return low > std::max(0.01, settings.collisionTolerance) ? translated(pose, direction, low) : pose;
}

double contactScore(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const std::vector<unsigned char>& placed,
    size_t partIndex,
    const Pose& pose) {
    const TransformedPart moving = transformPart(document.parts[partIndex], pose, static_cast<int>(partIndex));
    double score = 0.0;
    const double window = std::max(0.35, settings.partSpacing + 1.5);
    const AABB expanded = moving.bounds.expanded(window);
    for (size_t i = 0; i < placed.size() && i < document.parts.size() && i < poses.size(); ++i) {
        if (!placed[i] || i == partIndex) {
            continue;
        }
        const TransformedPart other = transformPart(document.parts[i], poses[i], static_cast<int>(i));
        if (!expanded.overlaps(other.bounds, window)) {
            continue;
        }
        const ClearanceResult clearance = minimumBoundaryDistance(moving, other, settings.partSpacing, settings.collisionTolerance);
        if (std::isfinite(clearance.minDistance)) {
            const double d = std::abs(clearance.minDistance - settings.partSpacing);
            if (d <= window) {
                score += 1.0 + (window - d) / window;
            }
        }
    }
    return score;
}

AABB usedBoundsForPlaced(const Document& document, const std::vector<Pose>& poses, const std::vector<unsigned char>& placed) {
    AABB used;
    for (size_t i = 0; i < placed.size() && i < document.parts.size() && i < poses.size(); ++i) {
        if (placed[i]) {
            used.include(transformedBounds(document.parts[i], poses[i]));
        }
    }
    return used;
}

std::vector<Vec2> regionAnchors(const EmptyRegion& region) {
    const double insetX = region.bounds.width() * 0.18;
    const double insetY = region.bounds.height() * 0.18;
    std::vector<Vec2> anchors{
        region.center,
        {region.bounds.min.x + insetX, region.bounds.min.y + insetY},
        {region.bounds.max.x - insetX, region.bounds.min.y + insetY},
        {region.bounds.min.x + insetX, region.bounds.max.y - insetY},
        {region.bounds.max.x - insetX, region.bounds.max.y - insetY},
        {region.bounds.center().x, region.bounds.min.y + insetY},
        {region.bounds.center().x, region.bounds.max.y - insetY},
        {region.bounds.min.x + insetX, region.bounds.center().y},
        {region.bounds.max.x - insetX, region.bounds.center().y}
    };
    const bool horizontal = region.bounds.width() >= region.bounds.height();
    for (double t : {0.25, 0.40, 0.60, 0.75}) {
        if (horizontal) {
            anchors.push_back({
                region.bounds.min.x + region.bounds.width() * t,
                region.center.y
            });
        } else {
            anchors.push_back({
                region.center.x,
                region.bounds.min.y + region.bounds.height() * t
            });
        }
    }
    return anchors;
}

bool nearPoseDuplicate(const Pose& a, const Pose& b, double positionTolerance, double angleTolerance) {
    return a.mirrored == b.mirrored &&
        std::abs(a.x - b.x) <= positionTolerance &&
        std::abs(a.y - b.y) <= positionTolerance &&
        std::abs(a.angleRadians - b.angleRadians) <= angleTolerance;
}

void appendAngleIfUnique(std::vector<double>& angles, double angle, double tolerance) {
    for (double existing : angles) {
        if (std::abs(existing - angle) <= tolerance) {
            return;
        }
    }
    angles.push_back(angle);
}

std::vector<double> limitedAngles(const std::vector<double>& rotations, double currentAngle, size_t limit) {
    std::vector<double> angles;
    angles.reserve(std::max<size_t>(1, limit));
    appendAngleIfUnique(angles, currentAngle, 1e-7);
    if (limit <= 1) {
        return angles;
    }
    if (rotations.size() <= limit) {
        for (double angle : rotations) {
            appendAngleIfUnique(angles, angle, 1e-7);
        }
        return angles;
    }
    const size_t remaining = limit - 1;
    for (size_t i = 0; i < remaining; ++i) {
        const size_t index = std::min(rotations.size() - 1, (i * rotations.size()) / remaining);
        appendAngleIfUnique(angles, rotations[index], 1e-7);
    }
    return angles;
}

std::vector<double> fitAngles(const EngineSettings& settings, const std::vector<double>& rotations, double currentAngle, size_t limit) {
    std::vector<double> angles;
    appendAngleIfUnique(angles, currentAngle, 1e-7);
    if (settings.allowRotation && settings.rotationMode != RotationMode::None) {
        const double rightAngles[] = {0.0, 90.0, 180.0, 270.0, 45.0, 135.0, 225.0, 315.0};
        for (double degrees : rightAngles) {
            appendAngleIfUnique(angles, radians(degrees), 1e-7);
        }
        for (double angle : rotations) {
            appendAngleIfUnique(angles, angle, 1e-7);
        }
        const double step = radians(std::max(0.001, settings.rotationStepDegrees));
        appendAngleIfUnique(angles, currentAngle + step, 1e-7);
        appendAngleIfUnique(angles, currentAngle - step, 1e-7);
    }
    if (angles.size() > limit) {
        std::vector<double> reduced;
        reduced.reserve(limit);
        appendAngleIfUnique(reduced, currentAngle, 1e-7);
        const size_t remaining = limit > 0 ? limit - 1 : 0;
        for (size_t i = 0; i < remaining && angles.size() > 1; ++i) {
            const size_t index = 1 + std::min(angles.size() - 2, (i * (angles.size() - 1)) / std::max<size_t>(1, remaining));
            appendAngleIfUnique(reduced, angles[index], 1e-7);
        }
        return reduced;
    }
    return angles.empty() ? limitedAngles(rotations, currentAngle, limit) : angles;
}

std::vector<bool> limitedMirrors(const std::vector<bool>& mirrors, bool currentMirror) {
    std::vector<bool> out;
    out.push_back(currentMirror);
    for (bool mirrored : mirrors) {
        if (std::find(out.begin(), out.end(), mirrored) == out.end()) {
            out.push_back(mirrored);
        }
    }
    return out;
}

std::vector<Vec2> collectAnchors(
    const Document& document,
    const EmptySpaceMap& map,
    const std::vector<Pose>& poses,
    const std::vector<unsigned char>& placed,
    const std::vector<Vec2>& extraAnchors,
    size_t regionLimit) {
    std::vector<Vec2> anchors;
    for (size_t i = 0; i < map.regions.size() && i < regionLimit; ++i) {
        std::vector<Vec2> region = regionAnchors(map.regions[i]);
        anchors.insert(anchors.end(), region.begin(), region.end());
    }
    anchors.insert(anchors.end(), extraAnchors.begin(), extraAnchors.end());
    const AABB placedUsed = usedBoundsForPlaced(document, poses, placed);
    if (placedUsed.isValid()) {
        anchors.push_back(placedUsed.center());
        anchors.push_back({placedUsed.min.x, placedUsed.center().y});
        anchors.push_back({placedUsed.max.x, placedUsed.center().y});
        anchors.push_back({placedUsed.center().x, placedUsed.min.y});
        anchors.push_back({placedUsed.center().x, placedUsed.max.y});
    }
    const size_t placedSampleLimit = document.parts.size() > 250 ? 4u :
        document.parts.size() > 100 ? 6u : 10u;
    std::vector<std::pair<double, size_t>> owners;
    owners.reserve(std::min(placed.size(), document.parts.size()));
    for (size_t owner = 0; owner < placed.size() && owner < document.parts.size() && owner < poses.size(); ++owner) {
        if (!placed[owner]) {
            continue;
        }
        const AABB box = transformedBounds(document.parts[owner], poses[owner]);
        double bestDistance = std::numeric_limits<double>::max();
        for (Vec2 anchor : anchors) {
            bestDistance = std::min(bestDistance, distance(box.center(), anchor));
        }
        owners.push_back({bestDistance - footprint(document.parts[owner]) * 0.0005, owner});
    }
    std::stable_sort(owners.begin(), owners.end(), [](const auto& a, const auto& b) {
        return a.first < b.first;
    });
    for (size_t i = 0; i < owners.size() && i < placedSampleLimit; ++i) {
        const size_t owner = owners[i].second;
        const TransformedPart fixed = transformPart(document.parts[owner], poses[owner], static_cast<int>(owner));
        std::vector<Vec2> points = samplePartBoundary(fixed, 3u);
        anchors.insert(anchors.end(), points.begin(), points.end());
    }
    if (anchors.size() > 48u) {
        anchors.resize(48u);
    }
    return anchors;
}

double placementScore(
    const Document& document,
    const EngineSettings& settings,
    const EmptySpaceMap& map,
    const std::vector<Pose>& poses,
    const std::vector<unsigned char>& placed,
    size_t partIndex,
    const Pose& pose,
    double contact) {
    AABB used = usedBoundsForPlaced(document, poses, placed);
    const AABB partBounds = transformedBounds(document.parts[partIndex], pose);
    used.include(partBounds);
    const LayoutShapeMetrics metrics = computeLayoutShapeMetrics(document, settings, used);
    const double areaWeight = used.area();
    const double towerPenalty = metrics.towerScore * std::max(1.0, document.totalPartArea()) * 0.9;
    double gapBonus = 0.0;
    for (size_t i = 0; i < map.regions.size() && i < 5u; ++i) {
        const EmptyRegion& region = map.regions[i];
        const double overlap = overlapArea(region.bounds, partBounds);
        if (overlap <= 0.0) {
            continue;
        }
        const double partArea = std::max(1.0, partBounds.area());
        const double fillRatio = std::min(1.0, overlap / partArea);
        gapBonus += fillRatio * std::min(partArea, region.area) * 1.8;
        if (containsBounds(region.bounds, partBounds, settings.partSpacing + settings.collisionTolerance)) {
            gapBonus += std::min(partArea, region.area) * 0.9;
        }
    }
    return areaWeight + towerPenalty - contact * 900.0 - gapBonus;
}

std::vector<Vec2> fitAnchorsForRegion(const Part& part, double angle, bool mirrored, const EmptyRegion& region, double spacing) {
    std::vector<Vec2> anchors;
    Pose orientation;
    orientation.angleRadians = angle;
    orientation.mirrored = mirrored;
    const AABB oriented = transformedBounds(part, orientation);
    const double halfWidth = oriented.width() * 0.5;
    const double halfHeight = oriented.height() * 0.5;
    const double minX = region.bounds.min.x + spacing + halfWidth;
    const double maxX = region.bounds.max.x - spacing - halfWidth;
    const double minY = region.bounds.min.y + spacing + halfHeight;
    const double maxY = region.bounds.max.y - spacing - halfHeight;
    auto clampAnchor = [&](Vec2 anchor) {
        if (minX <= maxX) {
            anchor.x = std::clamp(anchor.x, minX, maxX);
        }
        if (minY <= maxY) {
            anchor.y = std::clamp(anchor.y, minY, maxY);
        }
        anchors.push_back(anchor);
    };
    clampAnchor(region.center);
    clampAnchor({region.bounds.min.x + region.bounds.width() * 0.25, region.bounds.min.y + region.bounds.height() * 0.25});
    clampAnchor({region.bounds.max.x - region.bounds.width() * 0.25, region.bounds.min.y + region.bounds.height() * 0.25});
    clampAnchor({region.bounds.min.x + region.bounds.width() * 0.25, region.bounds.max.y - region.bounds.height() * 0.25});
    clampAnchor({region.bounds.max.x - region.bounds.width() * 0.25, region.bounds.max.y - region.bounds.height() * 0.25});
    const bool horizontal = region.bounds.width() >= region.bounds.height();
    for (double t : {0.20, 0.40, 0.60, 0.80}) {
        clampAnchor(horizontal
            ? Vec2{region.bounds.min.x + region.bounds.width() * t, region.center.y}
            : Vec2{region.center.x, region.bounds.min.y + region.bounds.height() * t});
    }
    return anchors;
}

bool bboxCanFitRegion(const Part& part, double angle, bool mirrored, const EmptyRegion& region, double spacing) {
    Pose orientation;
    orientation.angleRadians = angle;
    orientation.mirrored = mirrored;
    const AABB oriented = transformedBounds(part, orientation);
    const double availableWidth = std::max(0.0, region.bounds.width() - spacing * 2.0);
    const double availableHeight = std::max(0.0, region.bounds.height() - spacing * 2.0);
    return oriented.width() <= availableWidth + 1e-6 && oriented.height() <= availableHeight + 1e-6;
}

std::vector<Vec2> slideDirections(
    const Document& document,
    const EngineSettings& settings,
    const EmptySpaceMap& map,
    const std::vector<Pose>& poses,
    const std::vector<unsigned char>& placed,
    size_t partIndex,
    const Pose& pose) {
    std::vector<Vec2> directions{
        {-1.0, 0.0}, {1.0, 0.0}, {0.0, -1.0}, {0.0, 1.0},
        {-1.0, -1.0}, {-1.0, 1.0}, {1.0, -1.0}, {1.0, 1.0}
    };
    auto appendDirection = [&](Vec2 direction) {
        const double len = direction.length();
        if (len <= 1e-9) {
            return;
        }
        direction.x /= len;
        direction.y /= len;
        for (Vec2 existing : directions) {
            const double existingLen = existing.length();
            if (existingLen > 1e-9 &&
                std::abs(existing.x / existingLen - direction.x) <= 0.05 &&
                std::abs(existing.y / existingLen - direction.y) <= 0.05) {
                return;
            }
        }
        directions.push_back(direction);
    };

    const AABB partBounds = transformedBounds(document.parts[partIndex], pose);
    const Vec2 center = partBounds.center();
    switch (settings.placementStrategy) {
    case PlacementStrategy::TopLeft:
        appendDirection({-1.0, 1.0});
        break;
    case PlacementStrategy::BottomRight:
        appendDirection({1.0, -1.0});
        break;
    case PlacementStrategy::TopRight:
        appendDirection({1.0, 1.0});
        break;
    case PlacementStrategy::RightToLeft:
        appendDirection({-1.0, 0.0});
        break;
    case PlacementStrategy::TopToBottom:
        appendDirection({0.0, -1.0});
        break;
    case PlacementStrategy::BottomToTop:
        appendDirection({0.0, 1.0});
        break;
    case PlacementStrategy::CenterOut:
    case PlacementStrategy::OutsideIn:
        appendDirection({settings.sheetWidth * 0.5 - center.x, settings.sheetHeight * 0.5 - center.y});
        break;
    case PlacementStrategy::BottomLeft:
    case PlacementStrategy::LeftToRight:
    case PlacementStrategy::UserPoints:
    default:
        appendDirection({-1.0, -1.0});
        break;
    }

    for (size_t i = 0; i < map.regions.size() && i < 3u; ++i) {
        appendDirection(map.regions[i].center - center);
    }

    double bestDistance = std::numeric_limits<double>::max();
    Vec2 nearestCenter = center;
    for (size_t i = 0; i < placed.size() && i < document.parts.size() && i < poses.size(); ++i) {
        if (!placed[i] || i == partIndex) {
            continue;
        }
        const Vec2 otherCenter = transformedBounds(document.parts[i], poses[i]).center();
        const double d = distance(center, otherCenter);
        if (d < bestDistance) {
            bestDistance = d;
            nearestCenter = otherCenter;
        }
    }
    if (bestDistance < std::numeric_limits<double>::max()) {
        appendDirection(nearestCenter - center);
    }

    return directions;
}

bool appendCandidateIfUnique(
    std::vector<PlacementCandidate>& candidates,
    const PlacementCandidate& candidate,
    double positionTolerance,
    double angleTolerance) {
    for (const PlacementCandidate& existing : candidates) {
        if (nearPoseDuplicate(existing.pose, candidate.pose, positionTolerance, angleTolerance)) {
            return false;
        }
    }
    candidates.push_back(candidate);
    return true;
}

std::vector<PlacementCandidate> generatePlacementCandidates(
    const Document& document,
    const EngineSettings& settings,
    const EmptySpaceMap& map,
    const std::vector<double>& rotations,
    const std::vector<bool>& mirrors,
    const std::vector<Pose>& poses,
    const std::vector<unsigned char>& placed,
    const std::vector<Vec2>& extraAnchors,
    size_t partIndex,
    size_t regionLimit,
    size_t candidateLimit,
    CandidateDiagnostics* diagnostics) {
    std::vector<PlacementCandidate> out;
    if (partIndex >= document.parts.size() || partIndex >= poses.size()) {
        return out;
    }

    const Part& part = document.parts[partIndex];
    std::vector<Vec2> anchors = collectAnchors(document, map, poses, placed, extraAnchors, regionLimit);
    const size_t angleLimit = settings.performanceProfile == PerformanceProfile::Maximum ? 10u : 6u;
    const double averageArea = document.parts.empty() ? footprint(part) : document.totalPartArea() / static_cast<double>(document.parts.size());
    const bool smallOrMedium = footprint(part) <= averageArea * 1.25;
    const std::vector<double> angleSamples = fitAngles(settings, rotations, poses[partIndex].angleRadians, smallOrMedium ? angleLimit : std::max<size_t>(4u, angleLimit / 2u));
    const std::vector<bool> mirrorSamples = limitedMirrors(mirrors, poses[partIndex].mirrored);
    const double positionTolerance = std::max(0.05, settings.collisionTolerance * 2.0);
    constexpr double angleTolerance = 1e-5;
    const size_t effectiveCandidateLimit = smallOrMedium
        ? std::min<size_t>(candidateLimit * 2u, settings.performanceProfile == PerformanceProfile::Maximum ? 36u : 20u)
        : candidateLimit;

    for (double angle : angleSamples) {
        for (bool mirrored : mirrorSamples) {
            Pose orientation;
            orientation.angleRadians = angle;
            orientation.mirrored = mirrored;
            const TransformedPart local = transformPart(part, orientation, static_cast<int>(partIndex));
            std::vector<Vec2> movingPoints = samplePartBoundary(local, 2u);
            std::vector<Vec2> orientationAnchors = anchors;
            if (smallOrMedium) {
                for (size_t i = 0; i < map.regions.size() && i < regionLimit; ++i) {
                    if (!bboxCanFitRegion(part, angle, mirrored, map.regions[i], settings.partSpacing)) {
                        continue;
                    }
                    std::vector<Vec2> fitAnchors = fitAnchorsForRegion(part, angle, mirrored, map.regions[i], settings.partSpacing);
                    orientationAnchors.insert(orientationAnchors.end(), fitAnchors.begin(), fitAnchors.end());
                }
            }
            if (orientationAnchors.size() > 48u) {
                orientationAnchors.resize(48u);
            }
            bool enoughCandidates = false;
            for (Vec2 anchor : orientationAnchors) {
                std::vector<Pose> rawCandidates;
                rawCandidates.push_back(centerPose(part, angle, mirrored, anchor));
                for (Vec2 movingPoint : movingPoints) {
                    Pose pose = orientation;
                    pose.x = anchor.x - movingPoint.x;
                    pose.y = anchor.y - movingPoint.y;
                    rawCandidates.push_back(pose);
                }
                for (Pose candidate : rawCandidates) {
                    if (diagnostics) {
                        ++diagnostics->generated;
                    }
                    CandidateRejectReason reason = CandidateRejectReason::None;
                    if (!validateAgainstPlacedDetailed(document, settings, poses, placed, partIndex, candidate, &reason)) {
                        countReject(diagnostics, reason);
                        continue;
                    }
                    const std::vector<Vec2> directions = slideDirections(document, settings, map, poses, placed, partIndex, candidate);
                    Pose bestSlid = candidate;
                    double bestContact = contactScore(document, settings, poses, placed, partIndex, candidate);
                    for (Vec2 direction : directions) {
                        Pose slid = slideAgainstPlaced(document, settings, poses, placed, partIndex, candidate, direction, 160.0);
                        const double slidContact = contactScore(document, settings, poses, placed, partIndex, slid);
                        if (slidContact > bestContact + 1e-9) {
                            bestSlid = slid;
                            bestContact = slidContact;
                        }
                    }
                    PlacementCandidate candidateOut;
                    candidateOut.pose = bestSlid;
                    candidateOut.contact = bestContact;
                    candidateOut.localScore = placementScore(document, settings, map, poses, placed, partIndex, bestSlid, bestContact);
                    if (appendCandidateIfUnique(out, candidateOut, positionTolerance, angleTolerance)) {
                        if (diagnostics) {
                            ++diagnostics->valid;
                        }
                        if (out.size() >= effectiveCandidateLimit * 3u) {
                            enoughCandidates = true;
                            break;
                        }
                    }
                }
                if (enoughCandidates) {
                    break;
                }
            }
            if (enoughCandidates) {
                break;
            }
        }
    }

    std::stable_sort(out.begin(), out.end(), [](const PlacementCandidate& a, const PlacementCandidate& b) {
        if (std::abs(a.localScore - b.localScore) > 1e-9) {
            return a.localScore < b.localScore;
        }
        return a.contact > b.contact;
    });
    if (diagnostics) {
        diagnostics->maxCandidatesForPart = std::max(diagnostics->maxCandidatesForPart, out.size());
    }
    if (out.size() > effectiveCandidateLimit) {
        if (diagnostics) {
            diagnostics->beamPruned += out.size() - effectiveCandidateLimit;
        }
        out.resize(effectiveCandidateLimit);
    }
    if (out.empty() && diagnostics) {
        ++diagnostics->noCandidate;
    }
    return out;
}

std::vector<RepackSubset> buildSubsets(const Document& document, const EngineSettings& settings, const LayoutState& state, const EmptySpaceMap& map) {
    std::vector<RepackSubset> subsets;
    const size_t count = std::min(document.parts.size(), state.poses.size());
    if (count == 0 || !map.usedBounds.isValid()) {
        return subsets;
    }
    const size_t limit = document.parts.size() > 180
        ? (settings.performanceProfile == PerformanceProfile::Maximum ? 5u : 5u)
        : (settings.performanceProfile == PerformanceProfile::Maximum ? 6u : 5u);
    const double eps = std::max(2.0, settings.partSpacing + 2.0);
    const bool verticalTower = map.usedBounds.height() > map.usedBounds.width() * 2.4;
    auto fitScoreForRegion = [&](size_t partIndex, const EmptyRegion& region) {
        const Part& part = document.parts[partIndex];
        const double angles[] = {0.0, 90.0, 180.0, 270.0, 45.0, 135.0, 225.0, 315.0};
        double best = -1.0e9;
        const AABB current = transformedBounds(part, state.poses[partIndex]);
        const double epsBounds = std::max(2.0, settings.partSpacing + 2.0);
        double boundaryBoost = 0.0;
        if (std::abs(current.min.x - map.usedBounds.min.x) <= epsBounds ||
            std::abs(current.max.x - map.usedBounds.max.x) <= epsBounds) {
            boundaryBoost += 120.0;
        }
        if (std::abs(current.min.y - map.usedBounds.min.y) <= epsBounds ||
            std::abs(current.max.y - map.usedBounds.max.y) <= epsBounds) {
            boundaryBoost += 120.0;
        }
        for (double degrees : angles) {
            Pose orientation;
            orientation.angleRadians = radians(degrees);
            const AABB box = transformedBounds(part, orientation);
            const double widthSlack = region.bounds.width() - settings.partSpacing * 2.0 - box.width();
            const double heightSlack = region.bounds.height() - settings.partSpacing * 2.0 - box.height();
            const double areaRatio = footprint(part) / std::max(1.0, region.area);
            const double aspectPart = box.height() > 1e-9 ? box.width() / box.height() : 1.0;
            const double aspectRegion = region.bounds.height() > 1e-9 ? region.bounds.width() / region.bounds.height() : 1.0;
            const double aspectPenalty = std::abs(std::log(std::max(0.05, aspectPart) / std::max(0.05, aspectRegion))) * 12.0;
            const double fitBonus = widthSlack >= 0.0 && heightSlack >= 0.0 ? 200.0 : -80.0;
            const double slackPenalty = std::abs(widthSlack) + std::abs(heightSlack);
            best = std::max(best, fitBonus + areaRatio * 80.0 - slackPenalty - aspectPenalty);
        }
        return best + boundaryBoost - distance(current.center(), region.center) * 0.015;
    };

    auto rankedBy = [&](auto ranker) {
        std::vector<size_t> order(count);
        std::iota(order.begin(), order.end(), 0);
        std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
            return ranker(a) > ranker(b);
        });
        return order;
    };

    const size_t regionSubsetCount = std::min<size_t>(settings.performanceProfile == PerformanceProfile::Maximum ? 3u : 2u, map.regions.size());
    for (size_t regionIndex = 0; regionIndex < regionSubsetCount; ++regionIndex) {
        const EmptyRegion& region = map.regions[regionIndex];
        RepackSubset fitGap;
        fitGap.priority = 140.0 - static_cast<double>(regionIndex) * 8.0;
        const std::vector<size_t> order = rankedBy([&](size_t i) {
            return fitScoreForRegion(i, region);
        });
        for (size_t index : order) {
            appendUnique(fitGap.parts, index, limit);
        }
        subsets.push_back(std::move(fitGap));
    }

    RepackSubset boundary;
    boundary.priority = 90.0;
    const std::vector<size_t> boundaryOrder = rankedBy([&](size_t i) {
        const AABB box = transformedBounds(document.parts[i], state.poses[i]);
        double score = 0.0;
        if (std::abs(box.min.x - map.usedBounds.min.x) <= eps || std::abs(box.max.x - map.usedBounds.max.x) <= eps) {
            score += verticalTower ? 25.0 : 70.0;
        }
        if (std::abs(box.min.y - map.usedBounds.min.y) <= eps || std::abs(box.max.y - map.usedBounds.max.y) <= eps) {
            score += verticalTower ? 70.0 : 25.0;
        }
        score += footprint(document.parts[i]) * 0.001;
        return score;
    });
    for (size_t index : boundaryOrder) {
        appendUnique(boundary.parts, index, limit);
    }
    subsets.push_back(std::move(boundary));

    if (!map.regions.empty()) {
        const EmptyRegion& region = map.regions.front();
        RepackSubset aroundGap;
        aroundGap.priority = 120.0;
        const std::vector<size_t> order = rankedBy([&](size_t i) {
            const AABB box = transformedBounds(document.parts[i], state.poses[i]);
            return -distance(box.center(), region.center) + footprint(document.parts[i]) * 0.0002;
        });
        for (size_t index : order) {
            appendUnique(aroundGap.parts, index, limit);
        }
        subsets.push_back(std::move(aroundGap));
    }

    RepackSubset smallFillers;
    smallFillers.priority = 110.0;
    const std::vector<size_t> smallOrder = rankedBy([&](size_t i) {
        return -footprint(document.parts[i]);
    });
    for (size_t index : smallOrder) {
        appendUnique(smallFillers.parts, index, limit);
    }
    subsets.push_back(std::move(smallFillers));

    std::stable_sort(subsets.begin(), subsets.end(), [](const RepackSubset& a, const RepackSubset& b) {
        return a.priority > b.priority;
    });
    return subsets;
}

std::vector<Vec2> subsetAnchors(const Document& document, const LayoutState& state, const RepackSubset& subset) {
    std::vector<Vec2> anchors;
    AABB subsetBounds;
    for (size_t index : subset.parts) {
        if (index >= document.parts.size() || index >= state.poses.size()) {
            continue;
        }
        const AABB box = transformedBounds(document.parts[index], state.poses[index]);
        subsetBounds.include(box);
        anchors.push_back(box.center());
        anchors.push_back({box.min.x, box.center().y});
        anchors.push_back({box.max.x, box.center().y});
        anchors.push_back({box.center().x, box.min.y});
        anchors.push_back({box.center().x, box.max.y});
    }
    if (subsetBounds.isValid()) {
        anchors.push_back(subsetBounds.center());
        anchors.push_back(subsetBounds.min);
        anchors.push_back({subsetBounds.max.x, subsetBounds.min.y});
        anchors.push_back(subsetBounds.max);
        anchors.push_back({subsetBounds.min.x, subsetBounds.max.y});
    }
    if (anchors.size() > 56u) {
        anchors.resize(56u);
    }
    return anchors;
}

std::vector<size_t> orderSubset(const Document& document, const std::vector<size_t>& subset, int mode, uint32_t seed) {
    std::vector<size_t> order = subset;
    if (mode == 1) {
        std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
            return footprint(document.parts[a]) < footprint(document.parts[b]);
        });
    } else if (mode == 2) {
        std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
            return footprint(document.parts[a]) > footprint(document.parts[b]);
        });
    } else if (mode == 3) {
        std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
            const size_t ha = (a * 1103515245u + seed) & 0x7fffffffu;
            const size_t hb = (b * 1103515245u + seed) & 0x7fffffffu;
            return ha < hb;
        });
    }
    return order;
}

bool qualityBetter(const Document& document, const EngineSettings& settings, const LayoutState& candidate, const LayoutState& base) {
    if (!candidate.valid()) {
        return false;
    }
    const LayoutShapeMetrics before = computeLayoutShapeMetrics(document, settings, [&]() {
        AABB used;
        for (size_t i = 0; i < document.parts.size() && i < base.poses.size(); ++i) {
            used.include(transformedBounds(document.parts[i], base.poses[i]));
        }
        return used;
    }());
    const LayoutShapeMetrics after = computeLayoutShapeMetrics(document, settings, [&]() {
        AABB used;
        for (size_t i = 0; i < document.parts.size() && i < candidate.poses.size(); ++i) {
            used.include(transformedBounds(document.parts[i], candidate.poses[i]));
        }
        return used;
    }());
    if (candidate.utilization > base.utilization + 1e-6) {
        return true;
    }
    if (candidate.utilization + 0.005 >= base.utilization &&
        candidate.totalScore + std::max(1.0, std::abs(base.totalScore) * 1e-5) < base.totalScore) {
        return true;
    }
    if (after.towerScore + 0.08 < before.towerScore && candidate.utilization + 0.01 >= base.utilization) {
        return true;
    }
    return std::abs(candidate.utilization - base.utilization) <= 1e-6 &&
        candidate.totalScore + 1e-9 < base.totalScore;
}

void publishDiagnostics(SolverStats* stats, const CandidateDiagnostics& diagnostics) {
    if (!stats) {
        return;
    }
    stats->localRegionRepackCandidatesGenerated += diagnostics.generated;
    stats->localRegionRepackValidCandidates += diagnostics.valid;
    stats->localRegionRepackNoCandidate += diagnostics.noCandidate;
    stats->localRegionRepackCollisionReject += diagnostics.collisionReject;
    stats->localRegionRepackClearanceReject += diagnostics.clearanceReject;
    stats->localRegionRepackSheetReject += diagnostics.sheetReject;
    stats->localRegionRepackScoreReject += diagnostics.scoreReject;
    stats->localRegionRepackBeamPruned += diagnostics.beamPruned;
    stats->localRegionRepackFullValidationReject += diagnostics.fullValidationReject;
    stats->localRegionRepackMaxCandidatesForPart = std::max(stats->localRegionRepackMaxCandidatesForPart, diagnostics.maxCandidatesForPart);
}

LayoutState splitTowerToOpenSide(
    const Document& document,
    const EngineSettings& settings,
    const LayoutState& state,
    const EmptySpaceMap& map,
    const LayoutShapeMetrics& metrics,
    SolverStats* stats) {
    if (metrics.towerScore < 0.35 || !map.usedBounds.isValid()) {
        return state;
    }
    const bool verticalTower = map.usedBounds.height() > map.usedBounds.width() * 2.0;
    const double sheetMinX = document.sheet.origin.x + settings.margin;
    const double sheetMaxX = document.sheet.origin.x + document.sheet.width - settings.margin;
    const double sheetMinY = document.sheet.origin.y + settings.margin;
    const double sheetMaxY = document.sheet.origin.y + document.sheet.height - settings.margin;

    std::vector<size_t> order(document.parts.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        const AABB boxA = transformedBounds(document.parts[a], state.poses[a]);
        const AABB boxB = transformedBounds(document.parts[b], state.poses[b]);
        double scoreA = -footprint(document.parts[a]);
        double scoreB = -footprint(document.parts[b]);
        if (verticalTower) {
            if (std::abs(boxA.max.y - map.usedBounds.max.y) < 12.0 || std::abs(boxA.min.y - map.usedBounds.min.y) < 12.0) {
                scoreA += 100000.0;
            }
            if (std::abs(boxB.max.y - map.usedBounds.max.y) < 12.0 || std::abs(boxB.min.y - map.usedBounds.min.y) < 12.0) {
                scoreB += 100000.0;
            }
        } else {
            if (std::abs(boxA.max.x - map.usedBounds.max.x) < 12.0 || std::abs(boxA.min.x - map.usedBounds.min.x) < 12.0) {
                scoreA += 100000.0;
            }
            if (std::abs(boxB.max.x - map.usedBounds.max.x) < 12.0 || std::abs(boxB.min.x - map.usedBounds.min.x) < 12.0) {
                scoreB += 100000.0;
            }
        }
        return scoreA > scoreB;
    });

    const size_t moveCount = std::min<size_t>(verticalTower ? 2u : 2u, order.size());
    std::vector<size_t> moving(order.begin(), order.begin() + static_cast<std::ptrdiff_t>(moveCount));
    std::vector<Pose> poses = state.poses;
    std::vector<unsigned char> placed(poses.size(), 1);
    for (size_t index : moving) {
        if (index < placed.size()) {
            placed[index] = 0;
        }
    }

    const bool useRight = sheetMaxX - map.usedBounds.max.x >= map.usedBounds.min.x - sheetMinX;
    double cursor = verticalTower ? map.usedBounds.min.y : map.usedBounds.min.x;
    for (size_t index : moving) {
        if (index >= document.parts.size()) {
            continue;
        }
        const Part& part = document.parts[index];
        const Pose base = state.poses[index];
        const AABB local = transformedBounds(part, Pose{0.0, 0.0, base.angleRadians, base.mirrored});
        Pose candidate = base;
        if (verticalTower) {
            const double x = useRight
                ? std::min(sheetMaxX - local.width() * 0.5, map.usedBounds.max.x + settings.partSpacing + local.width() * 0.65)
                : std::max(sheetMinX + local.width() * 0.5, map.usedBounds.min.x - settings.partSpacing - local.width() * 0.65);
            const double y = std::min(sheetMaxY - local.height() * 0.5, std::max(sheetMinY + local.height() * 0.5, cursor + local.height() * 0.5));
            candidate = centerPose(part, base.angleRadians, base.mirrored, {x, y});
            cursor += local.height() + settings.partSpacing;
        } else {
            const double y = sheetMaxY - map.usedBounds.max.y >= map.usedBounds.min.y - sheetMinY
                ? std::min(sheetMaxY - local.height() * 0.5, map.usedBounds.max.y + settings.partSpacing + local.height() * 0.65)
                : std::max(sheetMinY + local.height() * 0.5, map.usedBounds.min.y - settings.partSpacing - local.height() * 0.65);
            const double x = std::min(sheetMaxX - local.width() * 0.5, std::max(sheetMinX + local.width() * 0.5, cursor + local.width() * 0.5));
            candidate = centerPose(part, base.angleRadians, base.mirrored, {x, y});
            cursor += local.width() + settings.partSpacing;
        }
        if (!validAgainstPlaced(document, settings, poses, placed, index, candidate)) {
            return state;
        }
        poses[index] = candidate;
        placed[index] = 1;
    }

    LayoutScore scorer;
    PenaltySystem penalties;
    LayoutState candidate = scorer.evaluate(document, settings, poses, &penalties);
    if (qualityBetter(document, settings, candidate, state)) {
        if (stats) {
            ++stats->localRegionRepackAccepted;
            ++stats->regionRepackAccepted;
            ++stats->acceptedMoves;
            ++stats->bestUpdates;
        }
        return candidate;
    }
    return state;
}

} // namespace

LayoutState LocalRegionRepack::improve(
    const Document& document,
    const EngineSettings& settings,
    LayoutState state,
    const std::atomic_bool& stopRequested,
    SolverStats* stats) const {
    if (settings.performanceProfile == PerformanceProfile::Fast || document.parts.empty() || state.poses.size() < document.parts.size()) {
        return state;
    }

    LayoutScore scorer;
    PenaltySystem penalties;
    state = scorer.evaluate(document, settings, state.poses, &penalties);
    if (!state.valid()) {
        return state;
    }

    const auto started = std::chrono::steady_clock::now();
    const double budgetSeconds = settings.timeLimitSeconds > 0.0
        ? std::min(10.0, std::max(3.0, settings.timeLimitSeconds * 0.25))
        : (settings.performanceProfile == PerformanceProfile::Maximum ? 12.0 : 5.0);
    auto expired = [&]() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() >= budgetSeconds;
    };

    EmptySpaceMap map = EmptySpaceAnalyzer{}.analyze(document, settings, state);
    const LayoutShapeMetrics initialMetrics = computeLayoutShapeMetrics(document, settings, map.usedBounds);
    const std::vector<RepackSubset> subsets = buildSubsets(document, settings, state, map);
    if (subsets.empty()) {
        return state;
    }

    PoseSampler sampler;
    std::vector<double> rotations = settings.allowRotation ? sampler.coarseRotationSamples(settings) : std::vector<double>{0.0};
    if (rotations.empty()) {
        rotations.push_back(0.0);
    }
    std::vector<bool> mirrors = sampler.mirrorSamples(settings);
    if (!settings.allowMirroring) {
        mirrors = {false};
    }
    const size_t regionLimit = settings.performanceProfile == PerformanceProfile::Maximum ? 4u : 3u;
    const int orderModes = settings.performanceProfile == PerformanceProfile::Maximum ? 4 : 2;
    CandidateDiagnostics diagnostics;

    for (const RepackSubset& subset : subsets) {
        if (stopRequested.load() || expired()) {
            break;
        }
        if (subset.parts.size() < 3) {
            continue;
        }
        if (stats) {
            ++stats->localRegionRepackSubsets;
        }
        const std::vector<Vec2> extraAnchors = subsetAnchors(document, state, subset);
        {
            std::vector<unsigned char> placed(state.poses.size(), 1);
            for (size_t partIndex : subset.parts) {
                if (stopRequested.load() || expired()) {
                    break;
                }
                if (partIndex >= document.parts.size() || partIndex >= state.poses.size()) {
                    continue;
                }
                placed.assign(state.poses.size(), 1);
                placed[partIndex] = 0;
                std::vector<PlacementCandidate> candidates = generatePlacementCandidates(
                    document,
                    settings,
                    map,
                    rotations,
                    mirrors,
                    state.poses,
                    placed,
                    extraAnchors,
                    partIndex,
                    regionLimit,
                    settings.performanceProfile == PerformanceProfile::Maximum ? 14u : 8u,
                    &diagnostics);
                if (stats) {
                    stats->localRegionRepackAttempts += candidates.size();
                }
                for (const PlacementCandidate& placement : candidates) {
                    std::vector<Pose> poses = state.poses;
                    poses[partIndex] = placement.pose;
                    PenaltySystem candidatePenalties;
                    LayoutState candidate = scorer.evaluate(document, settings, poses, &candidatePenalties);
                    if (!candidate.valid()) {
                        ++diagnostics.fullValidationReject;
                        continue;
                    }
                    if (!qualityBetter(document, settings, candidate, state)) {
                        ++diagnostics.scoreReject;
                        continue;
                    }
                    state = std::move(candidate);
                    publishDiagnostics(stats, diagnostics);
                    if (stats) {
                        ++stats->acceptedMoves;
                        ++stats->bestUpdates;
                        ++stats->localRegionRepackAccepted;
                        ++stats->regionRepackAccepted;
                    }
                    return state;
                }
            }
        }
        for (int mode = 0; mode < orderModes && !stopRequested.load() && !expired(); ++mode) {
            BeamNode root;
            root.poses = state.poses;
            root.placed.assign(root.poses.size(), 1);
            for (size_t index : subset.parts) {
                if (index < root.placed.size()) {
                    root.placed[index] = 0;
                }
            }
            const std::vector<size_t> order = orderSubset(document, subset.parts, mode, settings.randomSeed + static_cast<uint32_t>(mode * 97));
            root.depth = 0;
            root.contact = 0.0;
            root.score = usedBoundsForPlaced(document, root.poses, root.placed).area();

            const size_t beamWidth = settings.performanceProfile == PerformanceProfile::Maximum ? 4u : 3u;
            const size_t candidateLimit = settings.performanceProfile == PerformanceProfile::Maximum ? 12u : 8u;
            std::vector<BeamNode> beam;
            beam.push_back(std::move(root));
            bool failed = false;

            for (size_t partIndex : order) {
                if (stopRequested.load() || expired()) {
                    failed = true;
                    break;
                }
                if (partIndex >= document.parts.size()) {
                    continue;
                }
                std::vector<BeamNode> expanded;
                for (const BeamNode& node : beam) {
                    if (stopRequested.load() || expired()) {
                        failed = true;
                        break;
                    }
                    std::vector<PlacementCandidate> candidates = generatePlacementCandidates(
                        document,
                        settings,
                        map,
                        rotations,
                        mirrors,
                        node.poses,
                        node.placed,
                        extraAnchors,
                        partIndex,
                        regionLimit,
                        candidateLimit,
                        &diagnostics);
                    if (stats) {
                        stats->localRegionRepackAttempts += candidates.size();
                    }
                    for (const PlacementCandidate& candidate : candidates) {
                        BeamNode next = node;
                        next.poses[partIndex] = candidate.pose;
                        next.placed[partIndex] = 1;
                        next.depth = node.depth + 1;
                        next.contact = node.contact + candidate.contact;
                        next.score = candidate.localScore - next.contact * 120.0;
                        expanded.push_back(std::move(next));
                    }
                }
                if (failed) {
                    break;
                }
                if (expanded.empty()) {
                    failed = true;
                    break;
                }
                std::stable_sort(expanded.begin(), expanded.end(), [](const BeamNode& a, const BeamNode& b) {
                    if (std::abs(a.score - b.score) > 1e-9) {
                        return a.score < b.score;
                    }
                    return a.contact > b.contact;
                });
                if (expanded.size() > beamWidth) {
                    diagnostics.beamPruned += expanded.size() - beamWidth;
                    expanded.resize(beamWidth);
                }
                beam = std::move(expanded);
            }

            if (failed) {
                continue;
            }

            LayoutState bestAccepted;
            bool accepted = false;
            std::stable_sort(beam.begin(), beam.end(), [](const BeamNode& a, const BeamNode& b) {
                if (std::abs(a.score - b.score) > 1e-9) {
                    return a.score < b.score;
                }
                return a.contact > b.contact;
            });
            for (const BeamNode& node : beam) {
                PenaltySystem candidatePenalties;
                LayoutState candidate = scorer.evaluate(document, settings, node.poses, &candidatePenalties);
                if (!candidate.valid()) {
                    ++diagnostics.fullValidationReject;
                    continue;
                }
                if (!qualityBetter(document, settings, candidate, state)) {
                    ++diagnostics.scoreReject;
                    continue;
                }
                if (!accepted || qualityBetter(document, settings, candidate, bestAccepted) ||
                    candidate.totalScore + 1e-9 < bestAccepted.totalScore) {
                    bestAccepted = std::move(candidate);
                    accepted = true;
                }
            }
            if (accepted) {
                state = std::move(bestAccepted);
                publishDiagnostics(stats, diagnostics);
                if (stats) {
                    ++stats->acceptedMoves;
                    ++stats->bestUpdates;
                    ++stats->localRegionRepackAccepted;
                    ++stats->regionRepackAccepted;
                }
                return state;
            }
        }
    }

    publishDiagnostics(stats, diagnostics);
    return splitTowerToOpenSide(document, settings, state, map, initialMetrics, stats);
}

} // namespace nest
