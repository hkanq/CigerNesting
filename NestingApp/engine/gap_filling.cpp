#include "engine/gap_filling.h"

#include "core/math_utils.h"
#include "engine/free_space_analyzer.h"
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
    double footprint = 0.0;
};

struct BestMove {
    bool found = false;
    Pose pose;
    double score = std::numeric_limits<double>::max();
};

double partFootprint(const Part& part) {
    const double boundsArea = part.localBounds.area();
    if (part.area > 0.0) {
        return std::min(boundsArea > 0.0 ? boundsArea : part.area, part.area);
    }
    return boundsArea;
}

AABB orientedLocalBounds(const Part& part, double angleRadians, bool mirrored) {
    Pose pose;
    pose.angleRadians = angleRadians;
    pose.mirrored = mirrored;
    const Transform transform = pose.toTransform();
    AABB bounds;
    for (const Ring& ring : part.rings) {
        for (const Vec2& point : ring.points) {
            bounds.include(transform.apply(point));
        }
    }
    return bounds;
}

Pose centeredPose(const Part& part, double angleRadians, bool mirrored, Vec2 anchor) {
    const AABB bounds = orientedLocalBounds(part, angleRadians, mirrored);
    Pose pose;
    pose.angleRadians = angleRadians;
    pose.mirrored = mirrored;
    pose.x = anchor.x - bounds.center().x;
    pose.y = anchor.y - bounds.center().y;
    return pose;
}

void appendUniqueAngle(std::vector<double>& angles, double angle) {
    constexpr double fullTurn = 6.28318530717958647692;
    while (angle < 0.0) {
        angle += fullTurn;
    }
    while (angle >= fullTurn) {
        angle -= fullTurn;
    }
    for (double existing : angles) {
        if (std::abs(existing - angle) < 1e-9 || std::abs(std::abs(existing - angle) - fullTurn) < 1e-9) {
            return;
        }
    }
    angles.push_back(angle);
}

std::vector<double> rotationOptions(const EngineSettings& settings, const Pose& current) {
    std::vector<double> angles;
    appendUniqueAngle(angles, current.angleRadians);
    if (!settings.allowRotation || settings.rotationMode == RotationMode::None) {
        return angles;
    }

    PoseSampler sampler;
    std::vector<double> coarse = sampler.coarseRotationSamples(settings);
    const size_t limit = settings.performanceProfile == PerformanceProfile::Fast ? 2u :
        settings.performanceProfile == PerformanceProfile::Maximum ? 8u : 4u;
    if (coarse.size() > limit) {
        const size_t stride = std::max<size_t>(1, coarse.size() / limit);
        std::vector<double> reduced;
        for (size_t i = 0; i < coarse.size() && reduced.size() < limit; i += stride) {
            reduced.push_back(coarse[i]);
        }
        coarse = std::move(reduced);
    }
    for (double angle : coarse) {
        appendUniqueAngle(angles, angle);
    }

    const double step = degreesToRadians(std::max(0.001, settings.rotationStepDegrees));
    appendUniqueAngle(angles, current.angleRadians + step);
    appendUniqueAngle(angles, current.angleRadians - step);
    return angles;
}

std::vector<bool> mirrorOptions(const EngineSettings& settings, const Pose& current) {
    std::vector<bool> mirrors{current.mirrored};
    if (settings.allowMirroring) {
        mirrors.push_back(!current.mirrored);
    }
    return mirrors;
}

std::vector<Vec2> microOffsets(const EngineSettings& settings) {
    const double step = std::max(0.5, std::min(5.0, settings.partSpacing));
    std::vector<Vec2> offsets{{0.0, 0.0}, {step, 0.0}, {-step, 0.0}, {0.0, step}, {0.0, -step}};
    if (settings.performanceProfile == PerformanceProfile::Maximum) {
        offsets.push_back({step, step});
        offsets.push_back({-step, step});
        offsets.push_back({step, -step});
        offsets.push_back({-step, -step});
    }
    return offsets;
}

size_t targetPartLimit(const EngineSettings& settings, size_t partCount) {
    size_t limit = std::max<size_t>(1, (partCount * 30 + 99) / 100);
    if (partCount >= 3) {
        limit = std::max<size_t>(2, limit);
    }
    if (settings.performanceProfile == PerformanceProfile::Fast) {
        limit = std::min<size_t>(limit, 8);
    } else if (settings.performanceProfile == PerformanceProfile::Balanced) {
        limit = std::min<size_t>(limit, 32);
    } else {
        limit = std::min<size_t>(limit, 72);
    }
    return limit;
}

size_t evaluationBudget(const EngineSettings& settings, size_t partCount) {
    size_t budget = 8000;
    if (settings.performanceProfile == PerformanceProfile::Fast) {
        budget = 1800;
    } else if (settings.performanceProfile == PerformanceProfile::Maximum) {
        budget = 24000;
    }
    if (partCount > 300) {
        budget *= 2;
    }
    return budget;
}

size_t perPartEvaluationBudget(const EngineSettings& settings) {
    if (settings.performanceProfile == PerformanceProfile::Fast) {
        return 300;
    }
    if (settings.performanceProfile == PerformanceProfile::Maximum) {
        return 900;
    }
    return 600;
}

std::vector<size_t> smallPartOrder(const Document& document, const EngineSettings& settings) {
    std::vector<RankedPart> ranked;
    ranked.reserve(document.parts.size());
    for (size_t i = 0; i < document.parts.size(); ++i) {
        ranked.push_back({i, partFootprint(document.parts[i])});
    }
    std::sort(ranked.begin(), ranked.end(), [](const RankedPart& a, const RankedPart& b) {
        if (std::abs(a.footprint - b.footprint) > 1e-9) {
            return a.footprint < b.footprint;
        }
        return a.index < b.index;
    });
    const size_t limit = targetPartLimit(settings, document.parts.size());
    std::vector<size_t> order;
    order.reserve(std::min(limit, ranked.size()));
    for (size_t i = 0; i < ranked.size() && i < limit; ++i) {
        order.push_back(ranked[i].index);
    }
    return order;
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

int passCountForProfile(PerformanceProfile profile) {
    switch (profile) {
    case PerformanceProfile::Fast:
        return 1;
    case PerformanceProfile::Maximum:
        return 3;
    case PerformanceProfile::Balanced:
    default:
        return 2;
    }
}

} // namespace

LayoutState GapFilling::fillGaps(
    const Document& document,
    const EngineSettings& settings,
    LayoutState state,
    const PenaltySystem& penalties,
    const std::atomic_bool& stopRequested,
    SolverStats* stats) const {
    lastStats_ = {};
    if (document.parts.empty() || state.poses.size() < document.parts.size()) {
        return state;
    }

    LayoutScore scorer;
    state = scorer.evaluate(document, settings, state.poses, &penalties);
    if (!state.valid()) {
        return state;
    }

    const std::vector<size_t> targetParts = smallPartOrder(document, settings);
    const std::vector<Vec2> offsets = microOffsets(settings);
    const size_t maxEvaluations = evaluationBudget(settings, document.parts.size());
    const size_t maxPartEvaluations = perPartEvaluationBudget(settings);
    size_t evaluations = 0;

    for (int pass = 0; pass < passCountForProfile(settings.performanceProfile) && !stopRequested.load(); ++pass) {
        FreeSpaceAnalyzer analyzer;
        std::vector<FreeSpaceCandidate> anchors = analyzer.analyze(document, settings, state);
        lastStats_.generatedAnchors += anchors.size();
        for (const FreeSpaceCandidate& anchor : anchors) {
            if (anchor.kind == FreeSpaceCandidateKind::PartHole) {
                ++lastStats_.holeCandidates;
            } else if (anchor.kind == FreeSpaceCandidateKind::Concavity) {
                ++lastStats_.concavityCandidates;
            }
        }

        LayoutEvalCache evalCache;
        evalCache.rebuild(document, settings, state, &penalties);
        bool changed = false;

        for (size_t partIndex : targetParts) {
            if (stopRequested.load() || evaluations >= maxEvaluations || partIndex >= document.parts.size()) {
                break;
            }

            const Pose basePose = state.poses[partIndex];
            const std::vector<double> angles = rotationOptions(settings, basePose);
            const std::vector<bool> mirrors = mirrorOptions(settings, basePose);
            BestMove best;
            best.pose = basePose;
            best.score = state.totalScore;
            size_t partEvaluations = 0;

            for (const FreeSpaceCandidate& anchor : anchors) {
                if (stopRequested.load() || evaluations >= maxEvaluations || partEvaluations >= maxPartEvaluations) {
                    break;
                }
                if (anchor.sourcePart == partIndex) {
                    continue;
                }

                for (double angle : angles) {
                    for (bool mirrored : mirrors) {
                        Pose centered = centeredPose(document.parts[partIndex], angle, mirrored, anchor.anchor);
                        for (const Vec2& offset : offsets) {
                            if (evaluations >= maxEvaluations || partEvaluations >= maxPartEvaluations) {
                                break;
                            }
                            Pose candidate = centered;
                            candidate.x += offset.x;
                            candidate.y += offset.y;
                            if (samePose(candidate, basePose)) {
                                continue;
                            }

                            const DeltaMove move{partIndex, basePose, candidate};
                            const DeltaEvaluation trial = evaluateMoveDelta(document, settings, state, evalCache, move);
                            ++evaluations;
                            ++partEvaluations;
                            ++lastStats_.evaluatedCandidates;
                            if (stats) {
                                ++stats->evaluatedCandidates;
                            }
                            if (!trial.valid) {
                                classifyRejected(trial, stats);
                                continue;
                            }

                            const double priorityBonus = anchor.priority * 0.05;
                            const double rankedScore = trial.totalScore - priorityBonus;
                            if (rankedScore + 1e-9 < best.score) {
                                best.found = true;
                                best.pose = candidate;
                                best.score = rankedScore;
                            }
                        }
                    }
                }
            }

            if (!best.found) {
                continue;
            }

            std::vector<Pose> verifiedPoses = state.poses;
            verifiedPoses[partIndex] = best.pose;
            LayoutState verified = scorer.evaluate(document, settings, verifiedPoses, &penalties);
            if (verified.valid() && verified.totalScore + 1e-9 < state.totalScore) {
                const DeltaMove acceptedMove{partIndex, state.poses[partIndex], best.pose};
                state = std::move(verified);
                evalCache.updateAfterAcceptedMove(document, settings, state, acceptedMove, &penalties);
                changed = true;
                ++lastStats_.acceptedMoves;
                if (stats) {
                    ++stats->acceptedMoves;
                    ++stats->bestUpdates;
                }
            }
        }

        if (!changed) {
            break;
        }
    }

    return state;
}

} // namespace nest
