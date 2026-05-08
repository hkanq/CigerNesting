#include "engine/analytic_contact_candidate.h"

#include "core/math_utils.h"
#include "geometry/clearance.h"
#include "geometry/collision.h"
#include "geometry/transformed_shape.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace nest {
namespace {

struct RingFeature {
    Vec2 point;
    Vec2 next;
    bool hasEdge = false;
    bool isReflex = false;
    bool isHole = false;
    size_t sourcePart = static_cast<size_t>(-1);
    int sourceRing = -1;
};

bool samePose(const Pose& a, const Pose& b) {
    return a.mirrored == b.mirrored &&
        std::abs(a.x - b.x) <= 1e-5 &&
        std::abs(a.y - b.y) <= 1e-5 &&
        std::abs(a.angleRadians - b.angleRadians) <= 1e-8;
}

size_t usableCount(const std::vector<Vec2>& points) {
    if (points.size() > 2 && almostEqual(points.front(), points.back(), 1e-9)) {
        return points.size() - 1;
    }
    return points.size();
}

double signedArea(const std::vector<Vec2>& points) {
    const size_t count = usableCount(points);
    if (count < 3) {
        return 0.0;
    }
    double sum = 0.0;
    for (size_t i = 0; i < count; ++i) {
        const Vec2& a = points[i];
        const Vec2& b = points[(i + 1) % count];
        sum += a.x * b.y - b.x * a.y;
    }
    return sum * 0.5;
}

bool isReflexVertex(const std::vector<Vec2>& points, size_t index) {
    const size_t count = usableCount(points);
    if (count < 4 || index >= count) {
        return false;
    }
    const double area = signedArea(points);
    if (std::abs(area) <= 1e-9) {
        return false;
    }
    const double orientation = area > 0.0 ? 1.0 : -1.0;
    const Vec2 prev = points[(index + count - 1) % count];
    const Vec2 cur = points[index];
    const Vec2 next = points[(index + 1) % count];
    return orientation * cross(cur - prev, next - cur) < -1e-8;
}

std::vector<RingFeature> sampleRingFeatures(
    const TransformedRing& ring,
    size_t sourcePart,
    int sourceRing,
    size_t limit) {
    std::vector<RingFeature> features;
    const size_t count = usableCount(ring.points);
    if (count == 0 || limit == 0) {
        return features;
    }
    const size_t stride = std::max<size_t>(1, count / limit);
    for (size_t i = 0; i < count && features.size() < limit; i += stride) {
        RingFeature feature;
        feature.point = ring.points[i];
        feature.next = ring.points[(i + 1) % count];
        feature.hasEdge = count > 1 && distance(feature.point, feature.next) > 1e-9;
        feature.isReflex = !ring.isHole && isReflexVertex(ring.points, i);
        feature.isHole = ring.isHole;
        feature.sourcePart = sourcePart;
        feature.sourceRing = sourceRing;
        features.push_back(feature);
    }
    return features;
}

std::vector<RingFeature> partFeatures(const TransformedPart& part, size_t limitPerRing) {
    std::vector<RingFeature> features;
    for (size_t ringIndex = 0; ringIndex < part.rings.size(); ++ringIndex) {
        std::vector<RingFeature> ringFeatures = sampleRingFeatures(
            part.rings[ringIndex],
            static_cast<size_t>(std::max(0, part.partId)),
            static_cast<int>(ringIndex),
            part.rings[ringIndex].isHole ? limitPerRing + 4u : limitPerRing);
        features.insert(features.end(), ringFeatures.begin(), ringFeatures.end());
    }
    return features;
}

Ring rectangularSheetRing(const Document& document, const EngineSettings& settings) {
    Ring ring;
    const double left = document.sheet.origin.x + settings.margin;
    const double bottom = document.sheet.origin.y + settings.margin;
    const double right = document.sheet.origin.x + document.sheet.width - settings.margin;
    const double top = document.sheet.origin.y + document.sheet.height - settings.margin;
    ring.points = {{left, bottom}, {right, bottom}, {right, top}, {left, top}, {left, bottom}};
    return ring;
}

bool validateCandidate(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const AnalyticContactRequest& request,
    const Pose& pose,
    AnalyticContactStats* stats) {
    if (stats) {
        ++stats->generated;
    }
    if (request.movingPart >= document.parts.size()) {
        if (stats) {
            ++stats->rejectedSheet;
        }
        return false;
    }
    const Part& moving = document.parts[request.movingPart];
    if (!isPartInsideSheet(moving, pose, document.sheet, settings.collisionTolerance) ||
        !partRespectsSheetClearance(moving, pose, document.sheet, settings.margin, settings.collisionTolerance)) {
        if (stats) {
            ++stats->rejectedSheet;
        }
        return false;
    }

    const AABB movingBounds = transformedBounds(moving, pose).expanded(settings.partSpacing + settings.collisionTolerance);
    for (size_t other : request.fixedParts) {
        if (other == request.movingPart || other >= document.parts.size() || other >= poses.size()) {
            continue;
        }
        const AABB otherBounds = transformedBounds(document.parts[other], poses[other]);
        if (!movingBounds.overlaps(otherBounds, settings.collisionTolerance)) {
            continue;
        }
        if (partsCollide(moving, pose, document.parts[other], poses[other], settings.collisionTolerance)) {
            if (stats) {
                ++stats->rejectedCollision;
            }
            return false;
        }
        if (!partsRespectClearance(moving, pose, document.parts[other], poses[other], settings.partSpacing, settings.collisionTolerance)) {
            if (stats) {
                ++stats->rejectedClearance;
            }
            return false;
        }
    }
    if (stats) {
        ++stats->valid;
    }
    return true;
}

void appendUnique(
    std::vector<AnalyticContactCandidate>& out,
    const AnalyticContactCandidate& candidate,
    size_t limit) {
    for (const AnalyticContactCandidate& existing : out) {
        if (samePose(existing.pose, candidate.pose)) {
            return;
        }
    }
    out.push_back(candidate);
    if (out.size() > limit * 3u) {
        std::stable_sort(out.begin(), out.end(), [](const AnalyticContactCandidate& a, const AnalyticContactCandidate& b) {
            return a.priority > b.priority;
        });
        out.resize(limit * 2u);
    }
}

Pose poseFromPointAlignment(double angle, bool mirrored, Vec2 movingPoint, Vec2 fixedPoint) {
    Pose pose;
    pose.angleRadians = angle;
    pose.mirrored = mirrored;
    pose.x = fixedPoint.x - movingPoint.x;
    pose.y = fixedPoint.y - movingPoint.y;
    return pose;
}

Vec2 edgePoint(const RingFeature& edge, double t) {
    return edge.point + (edge.next - edge.point) * t;
}

Vec2 edgeNormal(const RingFeature& edge) {
    Vec2 dir = edge.next - edge.point;
    const double len = dir.length();
    if (len <= 1e-9) {
        return {0.0, 0.0};
    }
    dir = dir / len;
    return {-dir.y, dir.x};
}

double priorityForKind(AnalyticContactKind kind, bool fixedHole, bool fixedReflex) {
    double priority = 20.0;
    switch (kind) {
    case AnalyticContactKind::MovingVertexToFixedVertex:
        priority = 35.0;
        break;
    case AnalyticContactKind::MovingVertexToFixedEdge:
    case AnalyticContactKind::MovingEdgeToFixedVertex:
        priority = 44.0;
        break;
    case AnalyticContactKind::ConvexToConcaveVertex:
    case AnalyticContactKind::NotchCavity:
        priority = 72.0;
        break;
    case AnalyticContactKind::HoleBoundary:
        priority = 84.0;
        break;
    case AnalyticContactKind::SheetBoundary:
        priority = 38.0;
        break;
    case AnalyticContactKind::EdgeParallel:
        priority = 52.0;
        break;
    case AnalyticContactKind::RegionAnchor:
        priority = 46.0;
        break;
    case AnalyticContactKind::NfpPartPart:
        priority = 96.0;
        break;
    case AnalyticContactKind::NfpHoleBoundary:
        priority = 110.0;
        break;
    case AnalyticContactKind::InnerFitBoundary:
        priority = 92.0;
        break;
    }
    if (fixedHole) {
        priority += 22.0;
    }
    if (fixedReflex) {
        priority += 16.0;
    }
    return priority;
}

void tryAppend(
    std::vector<AnalyticContactCandidate>& out,
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const AnalyticContactRequest& request,
    Pose pose,
    AnalyticContactKind kind,
    size_t sourcePart,
    int sourceRing,
    double priority,
    AnalyticContactStats* stats) {
    if (!validateCandidate(document, settings, poses, request, pose, stats)) {
        return;
    }
    appendUnique(out, {pose, kind, sourcePart, sourceRing, priority}, std::max<size_t>(1, request.candidateLimit));
}

} // namespace

std::vector<AnalyticContactCandidate> AnalyticContactCandidateGenerator::generate(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const AnalyticContactRequest& request,
    AnalyticContactStats* stats) const {
    std::vector<AnalyticContactCandidate> out;
    if (request.movingPart >= document.parts.size()) {
        return out;
    }

    const size_t pointLimit = request.perOwnerPointLimit == 0 ? 8u : request.perOwnerPointLimit;
    std::vector<size_t> owners = request.fixedParts;
    if (owners.size() > request.ownerLimit) {
        const Vec2 movingCenter = request.movingPart < poses.size()
            ? transformedBounds(document.parts[request.movingPart], poses[request.movingPart]).center()
            : Vec2{settings.sheetWidth * 0.5, settings.sheetHeight * 0.5};
        std::stable_sort(owners.begin(), owners.end(), [&](size_t a, size_t b) {
            const double da = a < document.parts.size() && a < poses.size()
                ? distance(transformedBounds(document.parts[a], poses[a]).center(), movingCenter)
                : std::numeric_limits<double>::max();
            const double db = b < document.parts.size() && b < poses.size()
                ? distance(transformedBounds(document.parts[b], poses[b]).center(), movingCenter)
                : std::numeric_limits<double>::max();
            return da < db;
        });
        owners.resize(request.ownerLimit);
    }

    for (double angle : request.angles) {
        for (bool mirrored : request.mirrors) {
            Pose orientation;
            orientation.angleRadians = angle;
            orientation.mirrored = mirrored;
            const TransformedPart movingAtOrigin = transformPart(document.parts[request.movingPart], orientation, static_cast<int>(request.movingPart));
            const std::vector<RingFeature> movingFeatures = partFeatures(movingAtOrigin, pointLimit);
            if (movingFeatures.empty()) {
                continue;
            }

            if (request.includeRegionAnchors) {
                for (Vec2 anchor : request.regionAnchors) {
                    const AABB movingBox = movingAtOrigin.bounds;
                    const Vec2 center = movingBox.center();
                    Pose pose = orientation;
                    pose.x = anchor.x - center.x;
                    pose.y = anchor.y - center.y;
                    tryAppend(out, document, settings, poses, request, pose, AnalyticContactKind::RegionAnchor, static_cast<size_t>(-1), -1, 48.0, stats);
                }
            }

            if (request.includeSheetBoundary) {
                const Ring sheetRing = document.sheet.hasCustomProfile() && !document.sheet.profile().outerContour.points.empty()
                    ? document.sheet.profile().outerContour
                    : rectangularSheetRing(document, settings);
                TransformedRing fixedSheet;
                fixedSheet.points = sheetRing.points;
                fixedSheet.isHole = false;
                for (Vec2 p : fixedSheet.points) {
                    fixedSheet.bounds.include(p);
                }
                const std::vector<RingFeature> sheetFeatures = sampleRingFeatures(fixedSheet, static_cast<size_t>(-1), -1, 4u);
                for (const RingFeature& fixed : sheetFeatures) {
                    for (const RingFeature& moving : movingFeatures) {
                        tryAppend(out, document, settings, poses, request,
                            poseFromPointAlignment(angle, mirrored, moving.point, fixed.point),
                            AnalyticContactKind::SheetBoundary,
                            static_cast<size_t>(-1),
                            -1,
                            priorityForKind(AnalyticContactKind::SheetBoundary, false, false),
                            stats);
                        if (fixed.hasEdge) {
                            for (double t : {0.25, 0.50, 0.75}) {
                                tryAppend(out, document, settings, poses, request,
                                    poseFromPointAlignment(angle, mirrored, moving.point, edgePoint(fixed, t)),
                                    AnalyticContactKind::SheetBoundary,
                                    static_cast<size_t>(-1),
                                    -1,
                                    priorityForKind(AnalyticContactKind::SheetBoundary, false, false) + 2.0,
                                    stats);
                            }
                        }
                    }
                }
            }

            for (size_t owner : owners) {
                if (owner == request.movingPart || owner >= document.parts.size() || owner >= poses.size()) {
                    continue;
                }
                const TransformedPart fixedPart = transformPart(document.parts[owner], poses[owner], static_cast<int>(owner));
                const std::vector<RingFeature> fixedFeatures = partFeatures(fixedPart, pointLimit);
                for (const RingFeature& fixed : fixedFeatures) {
                    if (fixed.isHole && !request.includeHoleContacts) {
                        continue;
                    }
                    for (const RingFeature& moving : movingFeatures) {
                        const AnalyticContactKind vertexKind = fixed.isHole
                            ? AnalyticContactKind::HoleBoundary
                            : fixed.isReflex ? AnalyticContactKind::ConvexToConcaveVertex : AnalyticContactKind::MovingVertexToFixedVertex;
                        tryAppend(out, document, settings, poses, request,
                            poseFromPointAlignment(angle, mirrored, moving.point, fixed.point),
                            vertexKind,
                            owner,
                            fixed.sourceRing,
                            priorityForKind(vertexKind, fixed.isHole, fixed.isReflex),
                            stats);

                        if (fixed.hasEdge) {
                            for (double t : {0.25, 0.50, 0.75}) {
                                const Vec2 fixedPoint = edgePoint(fixed, t);
                                tryAppend(out, document, settings, poses, request,
                                    poseFromPointAlignment(angle, mirrored, moving.point, fixedPoint),
                                    fixed.isHole ? AnalyticContactKind::HoleBoundary : AnalyticContactKind::MovingVertexToFixedEdge,
                                    owner,
                                    fixed.sourceRing,
                                    priorityForKind(AnalyticContactKind::MovingVertexToFixedEdge, fixed.isHole, fixed.isReflex),
                                    stats);
                            }
                        }

                        if (moving.hasEdge) {
                            for (double t : {0.25, 0.50, 0.75}) {
                                const Vec2 movingPoint = edgePoint(moving, t);
                                tryAppend(out, document, settings, poses, request,
                                    poseFromPointAlignment(angle, mirrored, movingPoint, fixed.point),
                                    fixed.isReflex ? AnalyticContactKind::NotchCavity : AnalyticContactKind::MovingEdgeToFixedVertex,
                                    owner,
                                    fixed.sourceRing,
                                    priorityForKind(AnalyticContactKind::MovingEdgeToFixedVertex, fixed.isHole, fixed.isReflex),
                                    stats);
                            }
                        }

                        if (moving.hasEdge && fixed.hasEdge) {
                            Vec2 movingDir = moving.next - moving.point;
                            Vec2 fixedDir = fixed.next - fixed.point;
                            const double movingLen = movingDir.length();
                            const double fixedLen = fixedDir.length();
                            if (movingLen > 1e-9 && fixedLen > 1e-9) {
                                movingDir = movingDir / movingLen;
                                fixedDir = fixedDir / fixedLen;
                                const double parallel = std::abs(dot(movingDir, fixedDir));
                                if (parallel >= 0.94) {
                                    const Vec2 normal = edgeNormal(fixed);
                                    const Vec2 fixedMid = edgePoint(fixed, 0.5);
                                    const Vec2 movingMid = edgePoint(moving, 0.5);
                                    const double offset = settings.partSpacing;
                                    for (double side : {-1.0, 1.0}) {
                                        tryAppend(out, document, settings, poses, request,
                                            poseFromPointAlignment(angle, mirrored, movingMid, fixedMid + normal * (offset * side)),
                                            AnalyticContactKind::EdgeParallel,
                                            owner,
                                            fixed.sourceRing,
                                            priorityForKind(AnalyticContactKind::EdgeParallel, fixed.isHole, fixed.isReflex),
                                            stats);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    std::stable_sort(out.begin(), out.end(), [](const AnalyticContactCandidate& a, const AnalyticContactCandidate& b) {
        if (std::abs(a.priority - b.priority) > 1e-9) {
            return a.priority > b.priority;
        }
        return a.sourcePart < b.sourcePart;
    });
    if (out.size() > request.candidateLimit) {
        out.resize(request.candidateLimit);
    }
    return out;
}

} // namespace nest
