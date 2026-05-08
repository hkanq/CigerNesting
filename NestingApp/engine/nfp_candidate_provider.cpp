#include "engine/nfp_candidate_provider.h"

#include "geometry/clearance.h"
#include "geometry/collision.h"
#include "geometry/transformed_shape.h"
#include <algorithm>
#include <cmath>

namespace nest {
namespace {

struct NfpFeature {
    Vec2 point;
    Vec2 next;
    bool hasEdge = false;
    bool isHole = false;
    size_t sourcePart = static_cast<size_t>(-1);
    int sourceRing = -1;
};

size_t usableCount(const std::vector<Vec2>& points) {
    return points.size() > 2 && almostEqual(points.front(), points.back(), 1e-9) ? points.size() - 1u : points.size();
}

std::vector<NfpFeature> ringFeatures(const TransformedRing& ring, size_t sourcePart, int sourceRing, size_t limit) {
    std::vector<NfpFeature> features;
    const size_t count = usableCount(ring.points);
    if (count == 0 || limit == 0) {
        return features;
    }
    const size_t stride = std::max<size_t>(1u, count / limit);
    for (size_t i = 0; i < count && features.size() < limit; i += stride) {
        NfpFeature feature;
        feature.point = ring.points[i];
        feature.next = ring.points[(i + 1u) % count];
        feature.hasEdge = distance(feature.point, feature.next) > 1e-9;
        feature.isHole = ring.isHole;
        feature.sourcePart = sourcePart;
        feature.sourceRing = sourceRing;
        features.push_back(feature);
    }
    return features;
}

std::vector<NfpFeature> partFeatures(const TransformedPart& part, size_t limitPerRing) {
    std::vector<NfpFeature> features;
    for (size_t ringIndex = 0; ringIndex < part.rings.size(); ++ringIndex) {
        std::vector<NfpFeature> ring = ringFeatures(
            part.rings[ringIndex],
            static_cast<size_t>(std::max(0, part.partId)),
            static_cast<int>(ringIndex),
            part.rings[ringIndex].isHole ? limitPerRing + 4u : limitPerRing);
        features.insert(features.end(), ring.begin(), ring.end());
    }
    return features;
}

Vec2 edgePoint(const NfpFeature& feature, double t) {
    return feature.point + (feature.next - feature.point) * t;
}

Vec2 edgeNormal(const NfpFeature& feature) {
    Vec2 dir = feature.next - feature.point;
    const double len = dir.length();
    if (len <= 1e-9) {
        return {0.0, 0.0};
    }
    dir = dir / len;
    return {-dir.y, dir.x};
}

bool samePose(const Pose& a, const Pose& b) {
    return a.mirrored == b.mirrored &&
        std::abs(a.x - b.x) <= 1e-5 &&
        std::abs(a.y - b.y) <= 1e-5 &&
        std::abs(a.angleRadians - b.angleRadians) <= 1e-8;
}

bool validateCandidate(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const ContactCandidateRequest& request,
    const Pose& pose,
    ContactCandidateStats* stats) {
    if (stats != nullptr) {
        ++stats->generated;
    }
    if (request.movingPart >= document.parts.size()) {
        if (stats != nullptr) {
            ++stats->rejectedSheet;
        }
        return false;
    }
    const Part& moving = document.parts[request.movingPart];
    if (!isPartInsideSheet(moving, pose, document.sheet, settings.collisionTolerance) ||
        !partRespectsSheetClearance(moving, pose, document.sheet, settings.margin, settings.collisionTolerance)) {
        if (stats != nullptr) {
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
            if (stats != nullptr) {
                ++stats->rejectedCollision;
            }
            return false;
        }
        if (!partsRespectClearance(moving, pose, document.parts[other], poses[other], settings.partSpacing, settings.collisionTolerance)) {
            if (stats != nullptr) {
                ++stats->rejectedClearance;
            }
            return false;
        }
    }
    if (stats != nullptr) {
        ++stats->valid;
    }
    return true;
}

void appendLocalCandidate(std::vector<NfpCachedCandidate>& out, const NfpCachedCandidate& candidate, size_t limit) {
    for (const NfpCachedCandidate& existing : out) {
        if (samePose(existing.localPose, candidate.localPose)) {
            return;
        }
    }
    out.push_back(candidate);
    if (out.size() > limit * 4u) {
        std::stable_sort(out.begin(), out.end(), [](const NfpCachedCandidate& a, const NfpCachedCandidate& b) {
            return a.priority > b.priority;
        });
        out.resize(limit * 2u);
    }
}

NfpCacheKey makeKey(size_t moving, size_t fixed, const Pose& movingPose, const Pose& fixedPose, double spacing) {
    NfpCacheKey key;
    key.movingPartId = moving;
    key.fixedPartId = fixed;
    key.movingAngleBucket = nfpAngleBucket(movingPose.angleRadians);
    key.fixedAngleBucket = nfpAngleBucket(fixedPose.angleRadians);
    key.movingMirrored = movingPose.mirrored;
    key.fixedMirrored = fixedPose.mirrored;
    key.spacingBucket = nfpSpacingBucket(spacing);
    key.geometryVersion = nfpGeometryVersion(moving, fixed);
    return key;
}

NfpCacheValue buildLocalNfp(
    const Document& document,
    const EngineSettings& settings,
    size_t movingPart,
    size_t fixedPart,
    const Pose& movingOrientation,
    const Pose& fixedOrientation,
    size_t pointLimit,
    size_t candidateLimit) {
    NfpCacheValue value;
    if (movingPart >= document.parts.size() || fixedPart >= document.parts.size()) {
        return value;
    }
    const TransformedPart moving = transformPart(document.parts[movingPart], movingOrientation, static_cast<int>(movingPart));
    const TransformedPart fixed = transformPart(document.parts[fixedPart], fixedOrientation, static_cast<int>(fixedPart));
    const std::vector<NfpFeature> movingFeatures = partFeatures(moving, pointLimit);
    const std::vector<NfpFeature> fixedFeatures = partFeatures(fixed, pointLimit);
    const double spacing = settings.partSpacing;

    for (const NfpFeature& fixedFeature : fixedFeatures) {
        for (const NfpFeature& movingFeature : movingFeatures) {
            const AnalyticContactKind kind = fixedFeature.isHole ? AnalyticContactKind::NfpHoleBoundary : AnalyticContactKind::NfpPartPart;
            const double basePriority = fixedFeature.isHole ? 132.0 : 112.0;
            Pose vertexPose = movingOrientation;
            vertexPose.x = fixedFeature.point.x - movingFeature.point.x;
            vertexPose.y = fixedFeature.point.y - movingFeature.point.y;
            appendLocalCandidate(value.candidates, {vertexPose, kind, fixedPart, fixedFeature.sourceRing, basePriority}, candidateLimit);

            if (fixedFeature.hasEdge) {
                const Vec2 normal = edgeNormal(fixedFeature);
                for (double t : {0.0, 0.25, 0.50, 0.75, 1.0}) {
                    const Vec2 fixedPoint = edgePoint(fixedFeature, t);
                    for (double side : {-1.0, 1.0}) {
                        Pose pose = movingOrientation;
                        pose.x = fixedPoint.x + normal.x * spacing * side - movingFeature.point.x;
                        pose.y = fixedPoint.y + normal.y * spacing * side - movingFeature.point.y;
                        appendLocalCandidate(value.candidates, {pose, kind, fixedPart, fixedFeature.sourceRing, basePriority + 10.0}, candidateLimit);
                    }
                }
            }

            if (movingFeature.hasEdge) {
                for (double t : {0.25, 0.50, 0.75}) {
                    const Vec2 movingPoint = edgePoint(movingFeature, t);
                    Pose pose = movingOrientation;
                    pose.x = fixedFeature.point.x - movingPoint.x;
                    pose.y = fixedFeature.point.y - movingPoint.y;
                    appendLocalCandidate(value.candidates, {pose, kind, fixedPart, fixedFeature.sourceRing, basePriority + 7.0}, candidateLimit);
                }
            }

            if (movingFeature.hasEdge && fixedFeature.hasEdge) {
                Vec2 movingDir = movingFeature.next - movingFeature.point;
                Vec2 fixedDir = fixedFeature.next - fixedFeature.point;
                const double movingLen = movingDir.length();
                const double fixedLen = fixedDir.length();
                if (movingLen > 1e-9 && fixedLen > 1e-9) {
                    movingDir = movingDir / movingLen;
                    fixedDir = fixedDir / fixedLen;
                    if (std::abs(dot(movingDir, fixedDir)) >= 0.90) {
                        const Vec2 normal = edgeNormal(fixedFeature);
                        const Vec2 fixedMid = edgePoint(fixedFeature, 0.5);
                        const Vec2 movingMid = edgePoint(movingFeature, 0.5);
                        for (double side : {-1.0, 1.0}) {
                            Pose pose = movingOrientation;
                            pose.x = fixedMid.x + normal.x * spacing * side - movingMid.x;
                            pose.y = fixedMid.y + normal.y * spacing * side - movingMid.y;
                            appendLocalCandidate(value.candidates, {pose, AnalyticContactKind::EdgeParallel, fixedPart, fixedFeature.sourceRing, basePriority + 18.0}, candidateLimit);
                        }
                    }
                }
            }
        }
    }
    std::stable_sort(value.candidates.begin(), value.candidates.end(), [](const NfpCachedCandidate& a, const NfpCachedCandidate& b) {
        return a.priority > b.priority;
    });
    if (value.candidates.size() > candidateLimit * 3u) {
        value.candidates.resize(candidateLimit * 3u);
    }
    return value;
}

void appendValidated(
    std::vector<ContactCandidate>& out,
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const ContactCandidateRequest& request,
    Pose pose,
    AnalyticContactKind kind,
    size_t sourcePart,
    int sourceRing,
    double priority,
    ContactCandidateStats* stats) {
    if (!validateCandidate(document, settings, poses, request, pose, stats)) {
        return;
    }
    for (const ContactCandidate& existing : out) {
        if (samePose(existing.pose, pose)) {
            return;
        }
    }
    out.push_back({pose, kind, sourcePart, sourceRing, priority});
}

} // namespace

std::vector<ContactCandidate> NfpCandidateProvider::generatePartPartCandidates(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const ContactCandidateRequest& request,
    ContactCandidateStats* stats) const {
    std::vector<ContactCandidate> out;
    if (request.movingPart >= document.parts.size()) {
        return out;
    }
    std::vector<size_t> owners = request.fixedParts;
    if (owners.size() > request.ownerLimit) {
        const Vec2 movingCenter = request.movingPart < poses.size()
            ? transformedBounds(document.parts[request.movingPart], poses[request.movingPart]).center()
            : Vec2{};
        std::stable_sort(owners.begin(), owners.end(), [&](size_t a, size_t b) {
            const double da = a < document.parts.size() && a < poses.size()
                ? distance(transformedBounds(document.parts[a], poses[a]).center(), movingCenter)
                : 1e100;
            const double db = b < document.parts.size() && b < poses.size()
                ? distance(transformedBounds(document.parts[b], poses[b]).center(), movingCenter)
                : 1e100;
            return da < db;
        });
        owners.resize(request.ownerLimit);
    }

    const size_t pointLimit = std::max<size_t>(2u, request.perOwnerPointLimit + 2u);
    for (size_t owner : owners) {
        if (owner == request.movingPart || owner >= document.parts.size() || owner >= poses.size()) {
            continue;
        }
        Pose fixedOrientation = poses[owner];
        fixedOrientation.x = 0.0;
        fixedOrientation.y = 0.0;
        for (double angle : request.angles) {
            for (bool mirrored : request.mirrors) {
                Pose movingOrientation;
                movingOrientation.angleRadians = angle;
                movingOrientation.mirrored = mirrored;
                const NfpCacheKey key = makeKey(request.movingPart, owner, movingOrientation, fixedOrientation, settings.partSpacing);
                NfpCacheValue cached;
                const bool hit = cache_.find(key, cached);
                if (stats != nullptr) {
                    if (hit) {
                        ++stats->cacheHits;
                    } else {
                        ++stats->cacheMisses;
                    }
                }
                if (!hit) {
                    cached = buildLocalNfp(document, settings, request.movingPart, owner, movingOrientation, fixedOrientation, pointLimit, request.candidateLimit);
                    cache_.store(key, cached);
                }
                for (const NfpCachedCandidate& local : cached.candidates) {
                    Pose pose = local.localPose;
                    pose.x += poses[owner].x;
                    pose.y += poses[owner].y;
                    appendValidated(out, document, settings, poses, request, pose, local.kind, local.sourcePart, local.sourceRing, local.priority, stats);
                    if (out.size() >= request.candidateLimit) {
                        return out;
                    }
                }
            }
        }
    }
    return out;
}

std::vector<ContactCandidate> NfpCandidateProvider::generatePartSheetCandidates(
    const Document&, const EngineSettings&, const std::vector<Pose>&, const ContactCandidateRequest&, ContactCandidateStats*) const {
    return {};
}

std::vector<ContactCandidate> NfpCandidateProvider::generatePartHoleCandidates(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const ContactCandidateRequest& request,
    ContactCandidateStats* stats) const {
    ContactCandidateRequest holeRequest = request;
    holeRequest.includeHoleContacts = true;
    std::vector<ContactCandidate> candidates = generatePartPartCandidates(document, settings, poses, holeRequest, stats);
    candidates.erase(std::remove_if(candidates.begin(), candidates.end(), [](const ContactCandidate& candidate) {
        return candidate.kind != AnalyticContactKind::NfpHoleBoundary;
    }), candidates.end());
    return candidates;
}

std::vector<ContactCandidate> NfpCandidateProvider::generateCandidates(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const ContactCandidateRequest& request,
    ContactCandidateStats* stats) const {
    return generatePartPartCandidates(document, settings, poses, request, stats);
}

} // namespace nest
