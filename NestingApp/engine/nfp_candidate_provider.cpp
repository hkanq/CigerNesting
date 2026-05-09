#include "engine/nfp_candidate_provider.h"

#include "geometry/clearance.h"
#include "geometry/collision.h"
#include "geometry/no_fit_polygon.h"
#include "geometry/transformed_shape.h"
#include <algorithm>
#include <cmath>

namespace nest {
namespace {

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

NfpSolverCacheKey makeSolverKey(
    const Document& document,
    size_t moving,
    size_t fixed,
    const Pose& movingPose,
    const Pose& fixedPose,
    double spacing,
    double tolerance) {
    NfpSolverCacheKey key;
    key.movingPartId = moving;
    key.fixedPartId = fixed;
    key.movingAngleBucket = nfpSolverAngleBucket(movingPose.angleRadians);
    key.fixedAngleBucket = nfpSolverAngleBucket(fixedPose.angleRadians);
    key.movingMirrored = movingPose.mirrored;
    key.fixedMirrored = fixedPose.mirrored;
    key.spacingBucket = nfpSolverSpacingBucket(spacing);
    key.toleranceBucket = nfpSolverToleranceBucket(tolerance);
    key.geometryVersion = nfpSolverGeometryVersion(document.parts[moving], document.parts[fixed], moving, fixed);
    return key;
}

NfpSolverCacheValue buildRealNfpLoops(
    const Document& document,
    const EngineSettings& settings,
    size_t movingPart,
    size_t fixedPart,
    const Pose& movingOrientation,
    const Pose& fixedOrientation) {
    NoFitPolygonOptions options;
    options.spacing = settings.partSpacing;
    options.tolerance = settings.collisionTolerance;
    options.includeHoles = true;
    return toSolverCacheValue(buildNoFitPolygon(
        document.parts[movingPart],
        movingOrientation,
        document.parts[fixedPart],
        fixedOrientation,
        options));
}

void appendRealNfpLocalCandidates(
    std::vector<NfpCachedCandidate>& out,
    const NfpSolverCacheValue& loops,
    const Pose& movingOrientation,
    size_t fixedPart,
    size_t limit,
    Vec2 target) {
    const NoFitPolygonResult result = toNoFitPolygonResult(loops);
    for (const NoFitPolygonLoop& loop : result.loops) {
        NoFitPolygonResult singleLoop;
        singleLoop.loops.push_back(loop);
        singleLoop.componentCount = 1;
        std::vector<Pose> poses = sampleNoFitPolygonCandidates(singleLoop, movingOrientation, std::max<size_t>(8u, limit), target);
        const AnalyticContactKind kind = loop.fromHole ? AnalyticContactKind::NfpHoleBoundary : AnalyticContactKind::NfpPartPart;
        const double basePriority = loop.fromHole ? 168.0 : 150.0;
        for (const Pose& pose : poses) {
            const double targetDistance = distance({pose.x, pose.y}, target);
            appendLocalCandidate(out, {pose, kind, fixedPart, -1, basePriority + (loop.exactConvex ? 8.0 : 0.0) - targetDistance * 0.001}, limit);
            if (out.size() >= limit * 4u) {
                return;
            }
        }
    }
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
                const NfpSolverCacheKey key = makeSolverKey(
                    document,
                    request.movingPart,
                    owner,
                    movingOrientation,
                    fixedOrientation,
                    settings.partSpacing,
                    settings.collisionTolerance);
                NfpSolverCacheValue cached;
                const bool hit = solverCache_.find(key, cached);
                if (stats != nullptr) {
                    if (hit) {
                        ++stats->cacheHits;
                    } else {
                        ++stats->cacheMisses;
                    }
                }
                if (!hit) {
                    cached = buildRealNfpLoops(document, settings, request.movingPart, owner, movingOrientation, fixedOrientation);
                    solverCache_.store(key, cached);
                }
                if (stats != nullptr) {
                    stats->nfpLoopsGenerated += cached.loops.size();
                }
                NfpCacheValue sampled;
                const Vec2 target = !request.regionAnchors.empty()
                    ? request.regionAnchors.front() - Vec2{poses[owner].x, poses[owner].y}
                    : Vec2{};
                appendRealNfpLocalCandidates(sampled.candidates, cached, movingOrientation, owner, request.candidateLimit, target);
                if (stats != nullptr) {
                    stats->nfpLoopCandidatesGenerated += sampled.candidates.size();
                }
                for (const NfpCachedCandidate& local : sampled.candidates) {
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
    std::vector<ContactCandidate> out;
    if (request.movingPart >= document.parts.size()) {
        return out;
    }
    for (size_t owner : request.fixedParts) {
        if (owner == request.movingPart || owner >= document.parts.size() || owner >= poses.size()) {
            continue;
        }
        const TransformedPart fixed = transformPart(document.parts[owner], poses[owner], static_cast<int>(owner));
        for (const TransformedRing& hole : fixed.rings) {
            if (!hole.isHole || !hole.bounds.isValid()) {
                continue;
            }
            for (double angle : request.angles) {
                for (bool mirrored : request.mirrors) {
                    Pose orientation;
                    orientation.angleRadians = angle;
                    orientation.mirrored = mirrored;
                    const AABB movingBounds = transformedBounds(document.parts[request.movingPart], orientation);
                    if (!movingBounds.isValid()) {
                        continue;
                    }
                    if (movingBounds.width() > hole.bounds.width() + settings.collisionTolerance ||
                        movingBounds.height() > hole.bounds.height() + settings.collisionTolerance) {
                        continue;
                    }
                    const Vec2 translations[] = {
                        hole.bounds.center() - movingBounds.center(),
                        hole.bounds.min - movingBounds.min,
                        Vec2{hole.bounds.max.x - movingBounds.max.x, hole.bounds.min.y - movingBounds.min.y},
                        hole.bounds.max - movingBounds.max,
                        Vec2{hole.bounds.min.x - movingBounds.min.x, hole.bounds.max.y - movingBounds.max.y}
                    };
                    for (Vec2 translation : translations) {
                        Pose pose = orientation;
                        pose.x = translation.x;
                        pose.y = translation.y;
                        appendValidated(out, document, settings, poses, request, pose, AnalyticContactKind::NfpHoleBoundary, owner, -1, 182.0, stats);
                        if (out.size() >= request.candidateLimit) {
                            return out;
                        }
                    }
                }
            }
        }
    }
    return out;
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

