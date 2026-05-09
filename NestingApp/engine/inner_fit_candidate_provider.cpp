#include "engine/inner_fit_candidate_provider.h"

#include "geometry/clearance.h"
#include "geometry/collision.h"
#include "geometry/inner_fit_polygon.h"
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

void appendValidated(
    std::vector<ContactCandidate>& out,
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const ContactCandidateRequest& request,
    Pose pose,
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
    out.push_back({pose, AnalyticContactKind::InnerFitBoundary, static_cast<size_t>(-1), -1, priority});
}

} // namespace

std::vector<ContactCandidate> InnerFitCandidateProvider::generatePartSheetCandidates(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const ContactCandidateRequest& request,
    ContactCandidateStats* stats) const {
    std::vector<ContactCandidate> out;
    if (request.movingPart >= document.parts.size()) {
        return out;
    }

    for (double angle : request.angles) {
        for (bool mirrored : request.mirrors) {
            Pose orientation;
            orientation.angleRadians = angle;
            orientation.mirrored = mirrored;

            InnerFitPolygonOptions options;
            options.margin = settings.margin;
            options.tolerance = settings.collisionTolerance;
            const InnerFitPolygonResult ifp = buildInnerFitPolygon(document.parts[request.movingPart], orientation, document.sheet, options);
            if (stats != nullptr) {
                stats->ifpLoopsGenerated += ifp.loops.size();
            }
            const Vec2 target = request.regionAnchors.empty() ? Vec2{} : request.regionAnchors.front();
            const std::vector<Pose> ifpPoses = sampleInnerFitPolygonCandidates(ifp, orientation, request.candidateLimit, target);
            for (const Pose& pose : ifpPoses) {
                appendValidated(out, document, settings, poses, request, pose, ifp.exactRectangular ? 116.0 : 98.0, stats);
            }

            const AABB box = transformedBounds(document.parts[request.movingPart], orientation);
            for (Vec2 anchor : request.regionAnchors) {
                Pose pose = orientation;
                pose.x = anchor.x - box.center().x;
                pose.y = anchor.y - box.center().y;
                appendValidated(out, document, settings, poses, request, pose, 86.0, stats);
            }
        }
    }
    std::stable_sort(out.begin(), out.end(), [](const ContactCandidate& a, const ContactCandidate& b) {
        return a.priority > b.priority;
    });
    if (out.size() > request.candidateLimit) {
        out.resize(request.candidateLimit);
    }
    return out;
}
std::vector<ContactCandidate> InnerFitCandidateProvider::generatePartPartCandidates(
    const Document&, const EngineSettings&, const std::vector<Pose>&, const ContactCandidateRequest&, ContactCandidateStats*) const {
    return {};
}

std::vector<ContactCandidate> InnerFitCandidateProvider::generatePartHoleCandidates(
    const Document&, const EngineSettings&, const std::vector<Pose>&, const ContactCandidateRequest&, ContactCandidateStats*) const {
    return {};
}

std::vector<ContactCandidate> InnerFitCandidateProvider::generateCandidates(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    const ContactCandidateRequest& request,
    ContactCandidateStats* stats) const {
    return generatePartSheetCandidates(document, settings, poses, request, stats);
}

} // namespace nest


