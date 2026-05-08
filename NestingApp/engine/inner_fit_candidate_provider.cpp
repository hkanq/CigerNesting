#include "engine/inner_fit_candidate_provider.h"

#include "geometry/clearance.h"
#include "geometry/collision.h"
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

std::vector<Vec2> sheetAnchors(const Document& document, const EngineSettings& settings) {
    const double left = document.sheet.origin.x + settings.margin;
    const double bottom = document.sheet.origin.y + settings.margin;
    const double right = document.sheet.origin.x + document.sheet.width - settings.margin;
    const double top = document.sheet.origin.y + document.sheet.height - settings.margin;
    return {
        {left, bottom}, {right, bottom}, {left, top}, {right, top},
        {(left + right) * 0.5, bottom}, {(left + right) * 0.5, top},
        {left, (bottom + top) * 0.5}, {right, (bottom + top) * 0.5},
        {(left + right) * 0.5, (bottom + top) * 0.5}
    };
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
    const double left = document.sheet.origin.x + settings.margin;
    const double bottom = document.sheet.origin.y + settings.margin;
    const double right = document.sheet.origin.x + document.sheet.width - settings.margin;
    const double top = document.sheet.origin.y + document.sheet.height - settings.margin;

    for (double angle : request.angles) {
        for (bool mirrored : request.mirrors) {
            Pose orientation;
            orientation.angleRadians = angle;
            orientation.mirrored = mirrored;
            const AABB box = transformedBounds(document.parts[request.movingPart], orientation);
            const std::vector<Vec2> anchors = sheetAnchors(document, settings);
            for (Vec2 anchor : anchors) {
                Pose pose = orientation;
                const Vec2 center = box.center();
                pose.x = anchor.x - center.x;
                pose.y = anchor.y - center.y;
                appendValidated(out, document, settings, poses, request, pose, 92.0, stats);
            }
            const double w = box.width();
            const double h = box.height();
            const Vec2 boxMin = box.min;
            const Vec2 boxMax = box.max;
            const Vec2 corners[] = {
                {left - boxMin.x, bottom - boxMin.y},
                {right - boxMax.x, bottom - boxMin.y},
                {left - boxMin.x, top - boxMax.y},
                {right - boxMax.x, top - boxMax.y},
                {left - boxMin.x, (bottom + top - h) * 0.5 - boxMin.y},
                {(left + right - w) * 0.5 - boxMin.x, bottom - boxMin.y},
                {right - boxMax.x, (bottom + top - h) * 0.5 - boxMin.y},
                {(left + right - w) * 0.5 - boxMin.x, top - boxMax.y}
            };
            for (Vec2 translation : corners) {
                Pose pose = orientation;
                pose.x = translation.x;
                pose.y = translation.y;
                appendValidated(out, document, settings, poses, request, pose, 104.0, stats);
            }
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
