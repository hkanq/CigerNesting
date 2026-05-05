#include "engine/contact_packing.h"

#include "core/math_utils.h"
#include "engine/analytic_contact_candidate.h"
#include "engine/layout_eval_cache.h"
#include "engine/layout_score.h"
#include "engine/pose_sampler.h"
#include "geometry/transformed_shape.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace nest {
namespace {

struct RankedPart {
    size_t index = 0;
    double rank = 0.0;
};

bool samePose(const Pose& a, const Pose& b) {
    return std::abs(a.x - b.x) < 1e-9 &&
        std::abs(a.y - b.y) < 1e-9 &&
        std::abs(a.angleRadians - b.angleRadians) < 1e-9 &&
        a.mirrored == b.mirrored;
}

std::vector<Vec2> ringSamplePoints(const TransformedRing& ring, size_t stride) {
    std::vector<Vec2> points;
    if (ring.points.empty()) {
        return points;
    }
    const size_t usable = ring.points.size() > 2 && almostEqual(ring.points.front(), ring.points.back(), 1e-9)
        ? ring.points.size() - 1
        : ring.points.size();
    stride = std::max<size_t>(1, stride);
    for (size_t i = 0; i < usable; i += stride) {
        points.push_back(ring.points[i]);
    }
    if (points.empty() && usable > 0) {
        points.push_back(ring.points[0]);
    }
    return points;
}

std::vector<size_t> targetParts(const Document& document, const LayoutState& state, const EngineSettings& settings) {
    const size_t count = std::min(document.parts.size(), state.poses.size());
    AABB used;
    for (size_t i = 0; i < count; ++i) {
        used.include(transformedBounds(document.parts[i], state.poses[i]));
    }

    std::vector<RankedPart> ranked;
    ranked.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        const AABB box = transformedBounds(document.parts[i], state.poses[i]);
        const bool fringe = used.isValid() &&
            (box.center().x > used.min.x + used.width() * 0.65 ||
             box.center().y > used.min.y + used.height() * 0.65);
        const double area = std::max(1.0, document.parts[i].area > 0.0 ? document.parts[i].area : document.parts[i].localBounds.area());
        ranked.push_back({i, area - (fringe ? area * 0.55 : 0.0)});
    }
    std::sort(ranked.begin(), ranked.end(), [](const RankedPart& a, const RankedPart& b) {
        if (std::abs(a.rank - b.rank) > 1e-9) {
            return a.rank < b.rank;
        }
        return a.index < b.index;
    });
    size_t limit = settings.performanceProfile == PerformanceProfile::Maximum ? 48u :
        settings.performanceProfile == PerformanceProfile::Balanced ? 28u : 12u;
    if (document.parts.size() > 300) {
        limit = std::min<size_t>(limit, 26);
    }
    std::vector<size_t> out;
    for (size_t i = 0; i < ranked.size() && i < limit; ++i) {
        out.push_back(ranked[i].index);
    }
    return out;
}

std::vector<double> angleOptions(const EngineSettings& settings, const Pose& base) {
    std::vector<double> out{base.angleRadians};
    if (!settings.allowRotation || settings.rotationMode == RotationMode::None) {
        return out;
    }
    PoseSampler sampler;
    std::vector<double> coarse = sampler.coarseRotationSamples(settings);
    const size_t limit = settings.performanceProfile == PerformanceProfile::Maximum ? 8u :
        settings.performanceProfile == PerformanceProfile::Balanced ? 4u : 2u;
    for (size_t i = 0; i < coarse.size() && out.size() < limit + 1; ++i) {
        bool exists = false;
        for (double angle : out) {
            if (std::abs(angle - coarse[i]) < 1e-9) {
                exists = true;
                break;
            }
        }
        if (!exists) {
            out.push_back(coarse[i]);
        }
    }
    return out;
}

std::vector<bool> mirrorOptions(const EngineSettings& settings, const Pose& base) {
    std::vector<bool> out{base.mirrored};
    if (settings.allowMirroring) {
        out.push_back(!base.mirrored);
    }
    return out;
}

void classifyRejected(const DeltaEvaluation& evaluation, SolverStats* stats) {
    if (!stats) {
        return;
    }
    if (evaluation.collisionCount > 0) {
        ++stats->rejectedCollision;
    } else if (evaluation.spacingPenalty > 0.0) {
        ++stats->rejectedSpacing;
    } else if (evaluation.invalidPartCount > 0 || evaluation.sheetPenalty > 0.0) {
        ++stats->rejectedSheet;
    }
}

} // namespace

LayoutState ContactPacking::improve(
    const Document& document,
    const EngineSettings& settings,
    LayoutState state,
    const PenaltySystem& penalties,
    const std::atomic_bool& stopRequested,
    SolverStats* stats) const {
    if (document.parts.empty() || state.poses.size() < document.parts.size()) {
        return state;
    }

    LayoutScore scorer;
    state = scorer.evaluate(document, settings, state.poses, &penalties);
    if (!state.valid()) {
        return state;
    }

    LayoutEvalCache cache;
    cache.rebuild(document, settings, state, &penalties);
    std::vector<TransformedPart> transformed;
    transformed.reserve(document.parts.size());
    for (size_t i = 0; i < document.parts.size(); ++i) {
        transformed.push_back(transformPart(document.parts[i], state.poses[i], static_cast<int>(i)));
    }

    const size_t ownerLimit = settings.performanceProfile == PerformanceProfile::Maximum ? 40u :
        settings.performanceProfile == PerformanceProfile::Balanced ? 24u : 10u;
    const size_t sampleStride = settings.performanceProfile == PerformanceProfile::Maximum ? 2u : 3u;
    const size_t maxEvaluations = settings.performanceProfile == PerformanceProfile::Maximum ? 9000u :
        settings.performanceProfile == PerformanceProfile::Balanced ? 3200u : 900u;
    size_t evaluations = 0;

    const std::vector<size_t> targets = targetParts(document, state, settings);
    for (size_t movingIndex : targets) {
        if (stopRequested.load() || evaluations >= maxEvaluations || movingIndex >= document.parts.size()) {
            break;
        }

        const Pose base = state.poses[movingIndex];
        Pose bestPose = base;
        double bestScore = state.totalScore;
        bool found = false;

        std::vector<size_t> fixedParts;
        fixedParts.reserve(document.parts.size());
        for (size_t i = 0; i < document.parts.size(); ++i) {
            if (i != movingIndex) {
                fixedParts.push_back(i);
            }
        }
        AnalyticContactRequest request;
        request.movingPart = movingIndex;
        request.fixedParts = std::move(fixedParts);
        request.angles = angleOptions(settings, base);
        request.mirrors = mirrorOptions(settings, base);
        request.ownerLimit = ownerLimit;
        request.perOwnerPointLimit = settings.performanceProfile == PerformanceProfile::Maximum ? 8u : 5u;
        request.candidateLimit = settings.performanceProfile == PerformanceProfile::Maximum ? 160u : 80u;
        AnalyticContactStats analyticStats;
        AnalyticContactCandidateGenerator generator;
        const std::vector<AnalyticContactCandidate> analyticCandidates = generator.generate(document, settings, state.poses, request, &analyticStats);
        if (stats) {
            stats->analyticCandidatesGenerated += analyticStats.generated;
            stats->analyticCandidatesValid += analyticStats.valid;
            stats->contactCandidatesRejectedCollision += analyticStats.rejectedCollision;
            stats->contactCandidatesRejectedClearance += analyticStats.rejectedClearance;
            stats->contactCandidatesRejectedSheet += analyticStats.rejectedSheet;
        }
        for (const AnalyticContactCandidate& candidatePose : analyticCandidates) {
            if (evaluations >= maxEvaluations || samePose(candidatePose.pose, base)) {
                break;
            }
            const DeltaMove move{movingIndex, base, candidatePose.pose};
            const DeltaEvaluation delta = evaluateMoveDelta(document, settings, state, cache, move);
            ++evaluations;
            if (stats) {
                ++stats->evaluatedCandidates;
            }
            if (!delta.valid) {
                classifyRejected(delta, stats);
                continue;
            }
            if (delta.totalScore + 1e-9 < bestScore) {
                bestScore = delta.totalScore;
                bestPose = candidatePose.pose;
                found = true;
            } else if (stats) {
                ++stats->contactCandidatesRejectedScore;
            }
        }

        for (double angle : angleOptions(settings, base)) {
            for (bool mirrored : mirrorOptions(settings, base)) {
                Pose orientation;
                orientation.angleRadians = angle;
                orientation.mirrored = mirrored;
                const TransformedPart localMoving = transformPart(document.parts[movingIndex], orientation, static_cast<int>(movingIndex));
                std::vector<Vec2> movingSamples;
                for (const TransformedRing& ring : localMoving.rings) {
                    if (ring.points.size() >= 2) {
                        std::vector<Vec2> points = ringSamplePoints(ring, sampleStride);
                        movingSamples.insert(movingSamples.end(), points.begin(), points.end());
                    }
                }
                if (movingSamples.empty()) {
                    continue;
                }

                size_t ownersSeen = 0;
                for (size_t owner = 0; owner < transformed.size() && ownersSeen < ownerLimit; ++owner) {
                    if (owner == movingIndex || stopRequested.load()) {
                        continue;
                    }
                    ++ownersSeen;
                    for (const TransformedRing& ownerRing : transformed[owner].rings) {
                        if (ownerRing.points.size() < 2) {
                            continue;
                        }
                        const std::vector<Vec2> ownerSamples = ringSamplePoints(ownerRing, sampleStride);
                        for (Vec2 ownerPoint : ownerSamples) {
                            for (Vec2 movingPoint : movingSamples) {
                                if (evaluations >= maxEvaluations) {
                                    break;
                                }
                                Pose candidate;
                                candidate.angleRadians = angle;
                                candidate.mirrored = mirrored;
                                candidate.x = ownerPoint.x - movingPoint.x;
                                candidate.y = ownerPoint.y - movingPoint.y;
                                if (samePose(candidate, base)) {
                                    continue;
                                }
                                const DeltaMove move{movingIndex, base, candidate};
                                const DeltaEvaluation delta = evaluateMoveDelta(document, settings, state, cache, move);
                                ++evaluations;
                                if (stats) {
                                    ++stats->evaluatedCandidates;
                                }
                                if (!delta.valid) {
                                    classifyRejected(delta, stats);
                                    continue;
                                }
                                if (delta.totalScore + 1e-9 < bestScore) {
                                    bestScore = delta.totalScore;
                                    bestPose = candidate;
                                    found = true;
                                }
                            }
                        }
                    }
                }
            }
        }

        if (!found) {
            continue;
        }
        std::vector<Pose> poses = state.poses;
        poses[movingIndex] = bestPose;
        LayoutState verified = scorer.evaluate(document, settings, poses, &penalties);
        if (verified.valid() && verified.totalScore + 1e-9 < state.totalScore) {
            DeltaMove accepted{movingIndex, state.poses[movingIndex], bestPose};
            state = std::move(verified);
            cache.updateAfterAcceptedMove(document, settings, state, accepted, &penalties);
            transformed[movingIndex] = transformPart(document.parts[movingIndex], state.poses[movingIndex], static_cast<int>(movingIndex));
            if (stats) {
                ++stats->acceptedMoves;
                ++stats->activeMoveAcceptedTotal;
                ++stats->bestUpdates;
                ++stats->analyticCandidatesAccepted;
                ++stats->contourContactAccepted;
            }
        }
    }
    return state;
}

} // namespace nest
