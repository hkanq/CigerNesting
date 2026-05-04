#include "engine/rearrangement.h"

#include "engine/free_space_analyzer.h"
#include "engine/layout_score.h"
#include "geometry/clearance.h"
#include "geometry/collision.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace nest {
namespace {

struct RankedPart {
    size_t index = 0;
    double footprint = 0.0;
};

double partFootprint(const Part& part) {
    const double boundsArea = part.localBounds.area();
    if (part.area > 0.0) {
        return std::max(1.0, std::min(boundsArea > 0.0 ? boundsArea : part.area, part.area));
    }
    return std::max(1.0, boundsArea);
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

Pose centeredPose(const Part& part, const Pose& orientation, Vec2 anchor) {
    const AABB bounds = orientedLocalBounds(part, orientation.angleRadians, orientation.mirrored);
    Pose pose = orientation;
    pose.x = anchor.x - bounds.center().x;
    pose.y = anchor.y - bounds.center().y;
    return pose;
}

Vec2 boundsCenter(const Part& part, const Pose& pose) {
    return transformedBounds(part, pose).center();
}

std::vector<size_t> rankedParts(const Document& document, double percentile, size_t hardLimit) {
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
    const size_t percentileLimit = std::max<size_t>(1, static_cast<size_t>(std::ceil(static_cast<double>(ranked.size()) * percentile)));
    const size_t limit = std::min({ranked.size(), std::max<size_t>(2, percentileLimit), hardLimit});
    std::vector<size_t> out;
    out.reserve(limit);
    for (size_t i = 0; i < limit; ++i) {
        out.push_back(ranked[i].index);
    }
    return out;
}

bool poseRespectsLayout(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    size_t movingIndex,
    const Pose& pose) {
    if (movingIndex >= document.parts.size()) {
        return false;
    }
    if (!partRespectsSheetClearance(document.parts[movingIndex], pose, document.sheet, settings.margin, settings.collisionTolerance)) {
        return false;
    }
    for (size_t i = 0; i < document.parts.size() && i < poses.size(); ++i) {
        if (i == movingIndex) {
            continue;
        }
        if (partsCollide(document.parts[movingIndex], pose, document.parts[i], poses[i], settings.collisionTolerance)) {
            return false;
        }
        if (!partsRespectClearance(document.parts[movingIndex], pose, document.parts[i], poses[i], settings.partSpacing, settings.collisionTolerance)) {
            return false;
        }
    }
    return true;
}

std::vector<size_t> conflictingParts(
    const Document& document,
    const EngineSettings& settings,
    const std::vector<Pose>& poses,
    size_t movingIndex,
    const Pose& pose) {
    std::vector<size_t> conflicts;
    for (size_t i = 0; i < document.parts.size() && i < poses.size(); ++i) {
        if (i == movingIndex) {
            continue;
        }
        if (partsCollide(document.parts[movingIndex], pose, document.parts[i], poses[i], settings.collisionTolerance) ||
            !partsRespectClearance(document.parts[movingIndex], pose, document.parts[i], poses[i], settings.partSpacing, settings.collisionTolerance)) {
            conflicts.push_back(i);
        }
    }
    return conflicts;
}

bool tryAccept(
    const Document& document,
    const EngineSettings& settings,
    const PenaltySystem& penalties,
    LayoutState& state,
    const std::vector<Pose>& poses) {
    LayoutState trial = LayoutScore{}.evaluate(document, settings, poses, &penalties);
    if (trial.valid() && trial.totalScore + 1e-9 < state.totalScore) {
        state = std::move(trial);
        return true;
    }
    return false;
}

size_t swapPairBudget(const EngineSettings& settings) {
    if (settings.performanceProfile == PerformanceProfile::Fast) {
        return 24;
    }
    if (settings.performanceProfile == PerformanceProfile::Maximum) {
        return 220;
    }
    return 96;
}

size_t chainDepth(const EngineSettings& settings) {
    if (settings.performanceProfile == PerformanceProfile::Fast) {
        return 1;
    }
    if (settings.performanceProfile == PerformanceProfile::Maximum) {
        return 3;
    }
    return 2;
}

size_t maxAffectedParts(const EngineSettings& settings) {
    if (settings.performanceProfile == PerformanceProfile::Fast) {
        return 2;
    }
    if (settings.performanceProfile == PerformanceProfile::Maximum) {
        return 5;
    }
    return 3;
}

size_t clusterAttemptBudget(const EngineSettings& settings) {
    if (settings.performanceProfile == PerformanceProfile::Fast) {
        return 10;
    }
    if (settings.performanceProfile == PerformanceProfile::Maximum) {
        return 64;
    }
    return 28;
}

bool trySwapMoves(
    const Document& document,
    const EngineSettings& settings,
    const PenaltySystem& penalties,
    LayoutState& state,
    const std::atomic_bool& stopRequested,
    SolverStats* stats) {
    const size_t partLimit = settings.performanceProfile == PerformanceProfile::Maximum ? 42 :
        settings.performanceProfile == PerformanceProfile::Fast ? 12 : 24;
    const std::vector<size_t> parts = rankedParts(document, 0.65, std::min(partLimit, document.parts.size()));
    size_t attempts = 0;
    for (size_t ai = 0; ai < parts.size() && !stopRequested.load(); ++ai) {
        for (size_t bi = ai + 1; bi < parts.size() && !stopRequested.load(); ++bi) {
            if (attempts >= swapPairBudget(settings)) {
                return false;
            }
            const size_t a = parts[ai];
            const size_t b = parts[bi];
            if (a >= state.poses.size() || b >= state.poses.size()) {
                continue;
            }
            ++attempts;
            if (stats) {
                ++stats->swapAttempts;
            }

            const Pose poseA = state.poses[a];
            const Pose poseB = state.poses[b];
            const Vec2 centerA = boundsCenter(document.parts[a], poseA);
            const Vec2 centerB = boundsCenter(document.parts[b], poseB);

            std::vector<std::pair<Pose, Pose>> variants;
            Pose exactA = poseB;
            Pose exactB = poseA;
            variants.push_back({exactA, exactB});
            variants.push_back({centeredPose(document.parts[a], poseA, centerB), centeredPose(document.parts[b], poseB, centerA)});
            variants.push_back({centeredPose(document.parts[a], poseB, centerB), centeredPose(document.parts[b], poseA, centerA)});

            for (const auto& variant : variants) {
                std::vector<Pose> poses = state.poses;
                poses[a] = variant.first;
                poses[b] = variant.second;
                if (tryAccept(document, settings, penalties, state, poses)) {
                    if (stats) {
                        ++stats->swapAccepted;
                        ++stats->acceptedMoves;
                        ++stats->bestUpdates;
                    }
                    return true;
                }
            }
        }
    }
    return false;
}

std::vector<FreeSpaceCandidate> highValueAnchors(const std::vector<FreeSpaceCandidate>& anchors) {
    std::vector<FreeSpaceCandidate> out;
    for (const FreeSpaceCandidate& anchor : anchors) {
        if (anchor.kind == FreeSpaceCandidateKind::PartHole ||
            anchor.kind == FreeSpaceCandidateKind::Concavity ||
            anchor.kind == FreeSpaceCandidateKind::UsedBoundsGap) {
            out.push_back(anchor);
        }
    }
    if (out.empty()) {
        out = anchors;
    }
    return out;
}

bool relocatePartToFreeAnchor(
    const Document& document,
    const EngineSettings& settings,
    std::vector<Pose>& poses,
    size_t partIndex,
    const std::vector<FreeSpaceCandidate>& anchors,
    size_t skipSourcePart,
    size_t maxAnchorTests,
    SolverStats* stats) {
    if (partIndex >= document.parts.size() || partIndex >= poses.size()) {
        return false;
    }
    const Vec2 oldCenter = boundsCenter(document.parts[partIndex], poses[partIndex]);
    std::vector<FreeSpaceCandidate> ordered = anchors;
    std::sort(ordered.begin(), ordered.end(), [&](const FreeSpaceCandidate& a, const FreeSpaceCandidate& b) {
        const double da = distance(a.anchor, oldCenter) - a.priority * 0.10;
        const double db = distance(b.anchor, oldCenter) - b.priority * 0.10;
        return da < db;
    });
    size_t tests = 0;
    for (const FreeSpaceCandidate& anchor : ordered) {
        if (tests >= maxAnchorTests) {
            break;
        }
        if (anchor.sourcePart == partIndex || anchor.sourcePart == skipSourcePart) {
            continue;
        }
        ++tests;
        if (stats) {
            ++stats->evaluatedCandidates;
        }
        const Pose candidate = centeredPose(document.parts[partIndex], poses[partIndex], anchor.anchor);
        if (poseRespectsLayout(document, settings, poses, partIndex, candidate)) {
            poses[partIndex] = candidate;
            return true;
        }
    }
    return false;
}

bool tryEjectionChains(
    const Document& document,
    const EngineSettings& settings,
    const PenaltySystem& penalties,
    LayoutState& state,
    const std::atomic_bool& stopRequested,
    SolverStats* stats) {
    if (settings.performanceProfile == PerformanceProfile::Fast && document.parts.size() > 80) {
        return false;
    }
    FreeSpaceAnalyzer analyzer;
    const std::vector<FreeSpaceCandidate> anchors = highValueAnchors(analyzer.analyze(document, settings, state));
    if (anchors.empty()) {
        return false;
    }
    const std::vector<size_t> targets = rankedParts(document, 0.35, settings.performanceProfile == PerformanceProfile::Maximum ? 48 : 24);
    const size_t baseDepth = chainDepth(settings);
    const size_t maxAffected = maxAffectedParts(settings);
    const size_t perPartAnchorLimit = settings.performanceProfile == PerformanceProfile::Maximum ? 72 : settings.performanceProfile == PerformanceProfile::Fast ? 16 : 40;

    for (size_t partIndex : targets) {
        if (stopRequested.load() || partIndex >= state.poses.size()) {
            break;
        }
        size_t anchorTests = 0;
        for (const FreeSpaceCandidate& anchor : anchors) {
            if (stopRequested.load() || anchorTests >= perPartAnchorLimit) {
                break;
            }
            if (anchor.sourcePart == partIndex) {
                continue;
            }
            ++anchorTests;
            if (stats) {
                ++stats->chainAttempts;
                ++stats->evaluatedCandidates;
            }

            Pose targetPose = centeredPose(document.parts[partIndex], state.poses[partIndex], anchor.anchor);
            if (!partRespectsSheetClearance(document.parts[partIndex], targetPose, document.sheet, settings.margin, settings.collisionTolerance)) {
                continue;
            }

            std::vector<size_t> affected = conflictingParts(document, settings, state.poses, partIndex, targetPose);
            size_t adaptiveDepth = baseDepth;
            if (settings.performanceProfile != PerformanceProfile::Fast &&
                (anchor.kind == FreeSpaceCandidateKind::PartHole || anchor.kind == FreeSpaceCandidateKind::Concavity) &&
                state.utilization > 0.35) {
                adaptiveDepth = std::min(maxAffected, baseDepth + 1);
            }
            if (affected.empty() || affected.size() > maxAffected || affected.size() > adaptiveDepth) {
                continue;
            }

            std::vector<Pose> trialPoses = state.poses;
            trialPoses[partIndex] = targetPose;
            bool relocated = true;
            for (size_t affectedIndex : affected) {
                if (!relocatePartToFreeAnchor(document, settings, trialPoses, affectedIndex, anchors, partIndex, perPartAnchorLimit, stats)) {
                    relocated = false;
                    break;
                }
            }
            if (!relocated) {
                continue;
            }
            if (tryAccept(document, settings, penalties, state, trialPoses)) {
                if (stats) {
                    ++stats->chainAccepted;
                    ++stats->acceptedMoves;
                    ++stats->bestUpdates;
                }
                return true;
            }
        }
    }
    return false;
}

int horizontalCompressionSign(PlacementStrategy strategy) {
    switch (strategy) {
    case PlacementStrategy::BottomRight:
    case PlacementStrategy::TopRight:
    case PlacementStrategy::RightToLeft:
        return 1;
    default:
        return -1;
    }
}

int verticalCompressionSign(PlacementStrategy strategy) {
    switch (strategy) {
    case PlacementStrategy::TopLeft:
    case PlacementStrategy::TopRight:
    case PlacementStrategy::TopToBottom:
        return 1;
    default:
        return -1;
    }
}

Vec2 clusterDirection(const Document& document, const EngineSettings& settings, const LayoutState& state, const std::vector<size_t>& cluster, int axis) {
    if (settings.placementStrategy == PlacementStrategy::CenterOut ||
        settings.placementStrategy == PlacementStrategy::OutsideIn ||
        settings.placementStrategy == PlacementStrategy::UserPoints) {
        AABB clusterBounds;
        for (size_t index : cluster) {
            if (index < document.parts.size() && index < state.poses.size()) {
                clusterBounds.include(transformedBounds(document.parts[index], state.poses[index]));
            }
        }
        const Vec2 sheetCenter{
            document.sheet.origin.x + document.sheet.width * 0.5,
            document.sheet.origin.y + document.sheet.height * 0.5
        };
        const Vec2 center = clusterBounds.center();
        if (axis == 0) {
            return {sheetCenter.x > center.x ? 1.0 : -1.0, 0.0};
        }
        return {0.0, sheetCenter.y > center.y ? 1.0 : -1.0};
    }
    if (axis == 0) {
        return {static_cast<double>(horizontalCompressionSign(settings.placementStrategy)), 0.0};
    }
    return {0.0, static_cast<double>(verticalCompressionSign(settings.placementStrategy))};
}

std::vector<size_t> nearestCluster(const Document& document, const LayoutState& state, size_t seed, size_t maxSize) {
    std::vector<std::pair<double, size_t>> ranked;
    if (seed >= document.parts.size() || seed >= state.poses.size()) {
        return {};
    }
    const Vec2 center = boundsCenter(document.parts[seed], state.poses[seed]);
    ranked.reserve(document.parts.size());
    for (size_t i = 0; i < document.parts.size() && i < state.poses.size(); ++i) {
        ranked.push_back({distance(center, boundsCenter(document.parts[i], state.poses[i])), i});
    }
    std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) {
        return a.first < b.first;
    });
    const size_t count = std::min(maxSize, ranked.size());
    std::vector<size_t> cluster;
    cluster.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        cluster.push_back(ranked[i].second);
    }
    return cluster;
}

bool tryClusterCompaction(
    const Document& document,
    const EngineSettings& settings,
    const PenaltySystem& penalties,
    LayoutState& state,
    const std::atomic_bool& stopRequested,
    SolverStats* stats) {
    if (document.parts.size() < 3) {
        return false;
    }
    const size_t maxClusterSize = settings.performanceProfile == PerformanceProfile::Maximum ? 8 : 5;
    const std::vector<size_t> seeds = rankedParts(document, 0.50, clusterAttemptBudget(settings));
    const double baseStep = std::max(1.0, settings.partSpacing);
    const double steps[] = {baseStep * 4.0, baseStep * 2.0, baseStep};
    size_t attempts = 0;
    for (size_t seed : seeds) {
        if (stopRequested.load()) {
            break;
        }
        const std::vector<size_t> cluster = nearestCluster(document, state, seed, maxClusterSize);
        if (cluster.size() < 3) {
            continue;
        }
        for (int axis = 0; axis < 2; ++axis) {
            const Vec2 direction = clusterDirection(document, settings, state, cluster, axis);
            for (double step : steps) {
                if (attempts >= clusterAttemptBudget(settings)) {
                    return false;
                }
                ++attempts;
                if (stats) {
                    ++stats->clusterAttempts;
                    ++stats->evaluatedCandidates;
                }
                std::vector<Pose> poses = state.poses;
                for (size_t index : cluster) {
                    if (index < poses.size()) {
                        poses[index].x += direction.x * step;
                        poses[index].y += direction.y * step;
                    }
                }
                if (tryAccept(document, settings, penalties, state, poses)) {
                    if (stats) {
                        ++stats->clusterAccepted;
                        ++stats->acceptedMoves;
                        ++stats->bestUpdates;
                    }
                    return true;
                }

                AABB clusterBounds;
                std::vector<AABB> boundsByPart(document.parts.size());
                for (size_t index : cluster) {
                    if (index < document.parts.size() && index < state.poses.size()) {
                        const AABB bounds = transformedBounds(document.parts[index], state.poses[index]);
                        clusterBounds.include(bounds);
                        boundsByPart[index] = bounds;
                    }
                }
                if (!clusterBounds.isValid()) {
                    continue;
                }

                std::vector<Pose> shrinkPoses = state.poses;
                bool anyShrunk = false;
                for (size_t index : cluster) {
                    if (index >= shrinkPoses.size()) {
                        continue;
                    }
                    if (axis == 0) {
                        if (direction.x < 0.0 && boundsByPart[index].min.x > clusterBounds.min.x + 1e-6) {
                            shrinkPoses[index].x -= step;
                            anyShrunk = true;
                        } else if (direction.x > 0.0 && boundsByPart[index].max.x < clusterBounds.max.x - 1e-6) {
                            shrinkPoses[index].x += step;
                            anyShrunk = true;
                        }
                    } else {
                        if (direction.y < 0.0 && boundsByPart[index].min.y > clusterBounds.min.y + 1e-6) {
                            shrinkPoses[index].y -= step;
                            anyShrunk = true;
                        } else if (direction.y > 0.0 && boundsByPart[index].max.y < clusterBounds.max.y - 1e-6) {
                            shrinkPoses[index].y += step;
                            anyShrunk = true;
                        }
                    }
                }
                if (anyShrunk && tryAccept(document, settings, penalties, state, shrinkPoses)) {
                    if (stats) {
                        ++stats->clusterAccepted;
                        ++stats->acceptedMoves;
                        ++stats->bestUpdates;
                    }
                    return true;
                }
            }
        }
    }
    return false;
}

int passCount(PerformanceProfile profile) {
    switch (profile) {
    case PerformanceProfile::Fast:
        return 1;
    case PerformanceProfile::Maximum:
        return 4;
    case PerformanceProfile::Balanced:
    default:
        return 2;
    }
}

} // namespace

LayoutState Rearrangement::improve(
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

    for (int pass = 0; pass < passCount(settings.performanceProfile) && !stopRequested.load(); ++pass) {
        bool changed = false;
        changed = trySwapMoves(document, settings, penalties, state, stopRequested, stats) || changed;
        if (!stopRequested.load()) {
            changed = tryEjectionChains(document, settings, penalties, state, stopRequested, stats) || changed;
        }
        if (!stopRequested.load()) {
            changed = tryClusterCompaction(document, settings, penalties, state, stopRequested, stats) || changed;
        }
        if (!changed) {
            break;
        }
    }
    return state;
}

} // namespace nest
