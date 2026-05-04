#include "engine/small_part_filler.h"

#include "core/math_utils.h"
#include "engine/frontier_analyzer.h"
#include "engine/layout_eval_cache.h"
#include "engine/layout_score.h"
#include "engine/pose_sampler.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace nest {
namespace {

struct RankedPart {
    size_t index = 0;
    double area = 0.0;
    bool boundary = false;
};

double footprint(const Part& part) {
    if (part.area > 0.0) {
        return part.area;
    }
    return part.localBounds.area();
}

AABB orientedBounds(const Part& part, double angleRadians, bool mirrored) {
    Pose pose;
    pose.angleRadians = angleRadians;
    pose.mirrored = mirrored;
    return transformedBounds(part, pose);
}

Pose poseWithMinAt(const Part& part, double angleRadians, bool mirrored, Vec2 anchor) {
    const AABB box = orientedBounds(part, angleRadians, mirrored);
    Pose pose;
    pose.angleRadians = angleRadians;
    pose.mirrored = mirrored;
    pose.x = anchor.x - box.min.x;
    pose.y = anchor.y - box.min.y;
    return pose;
}

Pose poseWithCenterAt(const Part& part, double angleRadians, bool mirrored, Vec2 anchor) {
    const AABB box = orientedBounds(part, angleRadians, mirrored);
    Pose pose;
    pose.angleRadians = angleRadians;
    pose.mirrored = mirrored;
    pose.x = anchor.x - box.center().x;
    pose.y = anchor.y - box.center().y;
    return pose;
}

void addAngle(std::vector<double>& angles, double angle) {
    constexpr double fullTurn = 6.28318530717958647692;
    while (angle < 0.0) {
        angle += fullTurn;
    }
    while (angle >= fullTurn) {
        angle -= fullTurn;
    }
    for (double existing : angles) {
        if (std::abs(existing - angle) < 1e-9) {
            return;
        }
    }
    angles.push_back(angle);
}

std::vector<double> angleOptions(const EngineSettings& settings, const Pose& current) {
    std::vector<double> angles;
    addAngle(angles, current.angleRadians);
    if (!settings.allowRotation || settings.rotationMode == RotationMode::None) {
        return angles;
    }
    PoseSampler sampler;
    std::vector<double> coarse = sampler.coarseRotationSamples(settings);
    const size_t limit = settings.performanceProfile == PerformanceProfile::Maximum ? 8u :
        settings.performanceProfile == PerformanceProfile::Balanced ? 4u : 2u;
    if (coarse.size() > limit) {
        const size_t stride = std::max<size_t>(1, coarse.size() / limit);
        std::vector<double> reduced;
        for (size_t i = 0; i < coarse.size() && reduced.size() < limit; i += stride) {
            reduced.push_back(coarse[i]);
        }
        coarse = std::move(reduced);
    }
    for (double angle : coarse) {
        addAngle(angles, angle);
    }
    return angles;
}

std::vector<bool> mirrorOptions(const EngineSettings& settings, const Pose& current) {
    std::vector<bool> mirrors{current.mirrored};
    if (settings.allowMirroring) {
        mirrors.push_back(!current.mirrored);
    }
    return mirrors;
}

std::vector<Vec2> anchorVariants(const FrontierCandidate& candidate, const EngineSettings& settings) {
    std::vector<Vec2> anchors{candidate.anchor};
    const double spacing = settings.partSpacing;
    if (candidate.featureBounds.isValid()) {
        const AABB& box = candidate.featureBounds;
        anchors.push_back({box.min.x, box.max.y + spacing});
        anchors.push_back({box.max.x + spacing, box.min.y});
        anchors.push_back({box.min.x, box.min.y});
        anchors.push_back(box.center());
    }
    return anchors;
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

bool samePose(const Pose& a, const Pose& b) {
    return std::abs(a.x - b.x) < 1e-9 &&
        std::abs(a.y - b.y) < 1e-9 &&
        std::abs(a.angleRadians - b.angleRadians) < 1e-9 &&
        a.mirrored == b.mirrored;
}

std::vector<size_t> targetParts(const Document& document, const LayoutState& state, const EngineSettings& settings) {
    AABB used;
    const size_t count = std::min(document.parts.size(), state.poses.size());
    for (size_t i = 0; i < count; ++i) {
        used.include(transformedBounds(document.parts[i], state.poses[i]));
    }

    std::vector<RankedPart> ranked;
    ranked.reserve(count);
    const double eps = std::max(1.0, settings.partSpacing + settings.collisionTolerance);
    for (size_t i = 0; i < count; ++i) {
        const AABB box = transformedBounds(document.parts[i], state.poses[i]);
        const bool boundary = used.isValid() &&
            (std::abs(box.max.x - used.max.x) <= eps ||
             std::abs(box.max.y - used.max.y) <= eps ||
             box.center().x >= used.max.x - used.width() * 0.20 ||
             box.center().y >= used.max.y - used.height() * 0.20);
        const double edgeBias = used.isValid()
            ? (used.max.x - box.max.x) + (used.max.y - box.max.y) * 0.25
            : 0.0;
        ranked.push_back({i, footprint(document.parts[i]) + edgeBias, boundary});
    }

    std::sort(ranked.begin(), ranked.end(), [](const RankedPart& a, const RankedPart& b) {
        if (a.boundary != b.boundary) {
            return a.boundary && !b.boundary;
        }
        if (std::abs(a.area - b.area) > 1e-9) {
            return a.area < b.area;
        }
        return a.index < b.index;
    });

    size_t limit = settings.performanceProfile == PerformanceProfile::Fast ? 14u :
        settings.performanceProfile == PerformanceProfile::Maximum ? 96u : 48u;
    if (document.parts.size() > 300) {
        limit = std::min<size_t>(limit, 72);
    }
    limit = std::min(limit, ranked.size());

    std::vector<size_t> out;
    out.reserve(limit);
    for (size_t i = 0; i < limit; ++i) {
        out.push_back(ranked[i].index);
    }
    return out;
}

size_t candidateBudget(const EngineSettings& settings, size_t partCount) {
    size_t budget = settings.performanceProfile == PerformanceProfile::Fast ? 2500u :
        settings.performanceProfile == PerformanceProfile::Maximum ? 48000u : 18000u;
    if (partCount > 300) {
        budget = std::min<size_t>(budget, settings.performanceProfile == PerformanceProfile::Maximum ? 9000u : 4500u);
    } else if (partCount > 100) {
        budget = std::min<size_t>(budget, settings.performanceProfile == PerformanceProfile::Maximum ? 14000u : 7000u);
    }
    return budget;
}

} // namespace

LayoutState SmallPartFiller::fill(
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

    FrontierAnalyzer analyzer;
    std::vector<FrontierCandidate> frontiers = analyzer.analyze(document, settings, state);
    if (stats) {
        stats->frontierCandidates += frontiers.size();
    }
    if (frontiers.empty()) {
        return state;
    }

    const std::vector<size_t> parts = targetParts(document, state, settings);
    const size_t maxCandidates = candidateBudget(settings, document.parts.size());
    size_t evaluated = 0;
    bool changed = true;
    const int passes = settings.performanceProfile == PerformanceProfile::Maximum ? 3 :
        settings.performanceProfile == PerformanceProfile::Balanced ? 2 : 1;

    for (int pass = 0; pass < passes && changed && !stopRequested.load(); ++pass) {
        changed = false;
        LayoutEvalCache cache;
        cache.rebuild(document, settings, state, &penalties);

        for (size_t partIndex : parts) {
            if (partIndex >= document.parts.size() || stopRequested.load() || evaluated >= maxCandidates) {
                break;
            }

            const Pose base = state.poses[partIndex];
            const std::vector<double> angles = angleOptions(settings, base);
            const std::vector<bool> mirrors = mirrorOptions(settings, base);
            Pose bestPose = base;
            double bestScore = state.totalScore;
            bool found = false;

            for (const FrontierCandidate& frontier : frontiers) {
                if (frontier.sourcePart == partIndex) {
                    continue;
                }
                const std::vector<Vec2> anchors = anchorVariants(frontier, settings);
                for (double angle : angles) {
                    for (bool mirrored : mirrors) {
                        for (Vec2 anchor : anchors) {
                            if (evaluated >= maxCandidates || stopRequested.load()) {
                                break;
                            }

                            Pose candidates[2] = {
                                poseWithMinAt(document.parts[partIndex], angle, mirrored, anchor),
                                poseWithCenterAt(document.parts[partIndex], angle, mirrored, anchor)
                            };
                            for (const Pose& candidate : candidates) {
                                if (samePose(candidate, base)) {
                                    continue;
                                }
                                const DeltaMove move{partIndex, base, candidate};
                                const DeltaEvaluation trial = evaluateMoveDelta(document, settings, state, cache, move);
                                ++evaluated;
                                if (stats) {
                                    ++stats->evaluatedCandidates;
                                }
                                if (!trial.valid) {
                                    classifyRejected(trial, stats);
                                    continue;
                                }
                                const double priorityBonus = frontier.priority * 0.01;
                                if (trial.totalScore - priorityBonus + 1e-9 < bestScore) {
                                    bestScore = trial.totalScore - priorityBonus;
                                    bestPose = candidate;
                                    found = true;
                                }
                            }
                        }
                    }
                }
            }

            if (!found) {
                continue;
            }

            std::vector<Pose> verifiedPoses = state.poses;
            verifiedPoses[partIndex] = bestPose;
            LayoutState verified = scorer.evaluate(document, settings, verifiedPoses, &penalties);
            if (verified.valid() && verified.totalScore + 1e-9 < state.totalScore) {
                const DeltaMove accepted{partIndex, state.poses[partIndex], bestPose};
                state = std::move(verified);
                cache.updateAfterAcceptedMove(document, settings, state, accepted, &penalties);
                frontiers = analyzer.analyze(document, settings, state);
                if (stats) {
                    ++stats->acceptedMoves;
                    ++stats->bestUpdates;
                    ++stats->smallFillerAccepted;
                    stats->frontierCandidates += frontiers.size();
                }
                changed = true;
                break;
            }
        }
    }

    return state;
}

} // namespace nest
