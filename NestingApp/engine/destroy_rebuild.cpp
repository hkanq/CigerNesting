#include "engine/destroy_rebuild.h"

#include "engine/adaptive_acceptance.h"
#include "engine/analytic_contact_candidate.h"
#include "engine/empty_space_map.h"
#include "engine/layout_score.h"
#include "engine/layout_score_components.h"
#include "engine/pose_sampler.h"
#include "core/math_utils.h"
#include "geometry/transformed_shape.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

namespace nest {
namespace {

using Clock = std::chrono::steady_clock;

struct BeamNode {
    std::vector<Pose> poses;
    std::vector<size_t> placed;
    size_t depth = 0;
    double rank = std::numeric_limits<double>::max();
    double contactPriority = 0.0;
};

double partAreaScore(const Part& part) {
    return part.area > 0.0 ? part.area : part.localBounds.area();
}

bool containsIndex(const std::vector<size_t>& values, size_t index) {
    return std::find(values.begin(), values.end(), index) != values.end();
}

void appendUnique(std::vector<size_t>& values, size_t index, size_t limit) {
    if (values.size() >= limit || containsIndex(values, index)) {
        return;
    }
    values.push_back(index);
}

AABB usedBounds(const Document& document, const std::vector<Pose>& poses) {
    AABB box;
    for (size_t i = 0; i < document.parts.size() && i < poses.size(); ++i) {
        box.include(transformedBounds(document.parts[i], poses[i]));
    }
    return box;
}

std::vector<Vec2> regionAnchors(const EmptySpaceMap& map, const LayoutState& state, const Document& document, const std::vector<size_t>& subset) {
    std::vector<Vec2> anchors;
    for (size_t i = 0; i < map.regions.size() && i < 4u; ++i) {
        const EmptyRegion& region = map.regions[i];
        const double ix = region.bounds.width() * 0.18;
        const double iy = region.bounds.height() * 0.18;
        anchors.push_back(region.center);
        anchors.push_back({region.bounds.min.x + ix, region.bounds.min.y + iy});
        anchors.push_back({region.bounds.max.x - ix, region.bounds.min.y + iy});
        anchors.push_back({region.bounds.min.x + ix, region.bounds.max.y - iy});
        anchors.push_back({region.bounds.max.x - ix, region.bounds.max.y - iy});
        anchors.push_back({region.bounds.center().x, region.bounds.min.y + iy});
        anchors.push_back({region.bounds.center().x, region.bounds.max.y - iy});
        anchors.push_back({region.bounds.min.x + ix, region.bounds.center().y});
        anchors.push_back({region.bounds.max.x - ix, region.bounds.center().y});
    }
    for (size_t index : subset) {
        if (index < document.parts.size() && index < state.poses.size()) {
            const AABB box = transformedBounds(document.parts[index], state.poses[index]);
            anchors.push_back(box.center());
            anchors.push_back({box.min.x, box.center().y});
            anchors.push_back({box.max.x, box.center().y});
            anchors.push_back({box.center().x, box.min.y});
            anchors.push_back({box.center().x, box.max.y});
        }
    }
    if (anchors.size() > 72u) {
        anchors.resize(72u);
    }
    return anchors;
}

std::vector<double> angleSamples(const EngineSettings& settings, const Pose& base) {
    std::vector<double> angles{base.angleRadians};
    if (!settings.allowRotation || settings.rotationMode == RotationMode::None) {
        return angles;
    }
    PoseSampler sampler;
    for (double angle : sampler.coarseRotationSamples(settings)) {
        if (std::none_of(angles.begin(), angles.end(), [&](double existing) { return std::abs(existing - angle) < 1e-9; })) {
            angles.push_back(angle);
        }
    }
    if (settings.performanceProfile == PerformanceProfile::Maximum) {
        const double step = degreesToRadians(std::max(0.001, settings.rotationStepDegrees));
        angles.push_back(base.angleRadians + step);
        angles.push_back(base.angleRadians - step);
    }
    if (angles.size() > 10u) {
        angles.resize(10u);
    }
    return angles;
}

std::vector<bool> mirrorSamples(const EngineSettings& settings, const Pose& base) {
    std::vector<bool> mirrors{base.mirrored};
    if (settings.allowMirroring) {
        mirrors.push_back(!base.mirrored);
    }
    return mirrors;
}

double fitScore(size_t partIndex, const Document& document, const LayoutState& state, const EmptySpaceMap& map) {
    const AABB box = transformedBounds(document.parts[partIndex], state.poses[partIndex]);
    double score = 0.0;
    if (map.usedBounds.isValid()) {
        const double eps = 6.0;
        if (std::abs(box.min.x - map.usedBounds.min.x) <= eps || std::abs(box.max.x - map.usedBounds.max.x) <= eps) {
            score += 120.0;
        }
        if (std::abs(box.min.y - map.usedBounds.min.y) <= eps || std::abs(box.max.y - map.usedBounds.max.y) <= eps) {
            score += 120.0;
        }
    }
    if (!map.regions.empty()) {
        const EmptyRegion& region = map.regions.front();
        const double areaRatio = partAreaScore(document.parts[partIndex]) / std::max(1.0, region.area);
        score += std::min(1.0, areaRatio) * 90.0;
        score -= distance(box.center(), region.center) * 0.015;
        if (box.width() <= region.bounds.width() && box.height() <= region.bounds.height()) {
            score += 80.0;
        }
    }
    score += (1.0 / std::max(1.0, partAreaScore(document.parts[partIndex]))) * 2500.0;
    return score;
}

std::vector<std::vector<size_t>> buildSubsets(const Document& document, const LayoutState& state, const EmptySpaceMap& map, const EngineSettings& settings) {
    std::vector<std::vector<size_t>> subsets;
    const size_t count = std::min(document.parts.size(), state.poses.size());
    if (count == 0) {
        return subsets;
    }
    const size_t subsetLimit = settings.performanceProfile == PerformanceProfile::Maximum
        ? (count >= 300 ? 16u : 14u)
        : (count >= 300 ? 10u : 9u);
    std::vector<size_t> order(count);
    std::iota(order.begin(), order.end(), 0);

    std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return fitScore(a, document, state, map) > fitScore(b, document, state, map);
    });
    std::vector<size_t> fitSubset;
    for (size_t index : order) {
        appendUnique(fitSubset, index, subsetLimit);
    }
    subsets.push_back(std::move(fitSubset));

    if (!map.regions.empty()) {
        const EmptyRegion& region = map.regions.front();
        std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
            const AABB boxA = transformedBounds(document.parts[a], state.poses[a]);
            const AABB boxB = transformedBounds(document.parts[b], state.poses[b]);
            const double scoreA = -distance(boxA.center(), region.center) + (boxA.width() <= region.bounds.width() && boxA.height() <= region.bounds.height() ? 100.0 : 0.0);
            const double scoreB = -distance(boxB.center(), region.center) + (boxB.width() <= region.bounds.width() && boxB.height() <= region.bounds.height() ? 100.0 : 0.0);
            return scoreA > scoreB;
        });
        std::vector<size_t> nearGap;
        for (size_t index : order) {
            appendUnique(nearGap, index, subsetLimit);
        }
        subsets.push_back(std::move(nearGap));
    }

    std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return partAreaScore(document.parts[a]) < partAreaScore(document.parts[b]);
    });
    std::vector<size_t> small;
    for (size_t index : order) {
        appendUnique(small, index, subsetLimit);
    }
    subsets.push_back(std::move(small));
    return subsets;
}

double nodeRank(const Document& document, const EngineSettings& settings, const std::vector<Pose>& poses, double contactPriority) {
    const AABB used = usedBounds(document, poses);
    const LayoutShapeMetrics metrics = computeLayoutShapeMetrics(document, settings, used);
    return used.area() + metrics.towerScore * std::max(1.0, document.totalPartArea()) * 2.2 - contactPriority * 450.0;
}

std::vector<size_t> fixedPartsFor(const Document& document, const std::vector<size_t>& subset, const std::vector<size_t>& placedSubset) {
    std::vector<size_t> fixed;
    fixed.reserve(document.parts.size());
    for (size_t i = 0; i < document.parts.size(); ++i) {
        if (!containsIndex(subset, i) || containsIndex(placedSubset, i)) {
            fixed.push_back(i);
        }
    }
    return fixed;
}

bool betterLayout(const LayoutState& candidate, const LayoutState& incumbent) {
    if (!candidate.valid()) {
        return false;
    }
    if (!incumbent.valid()) {
        return true;
    }
    if (candidate.utilization > incumbent.utilization + 1e-6) {
        return true;
    }
    return candidate.totalScore + 1e-9 < incumbent.totalScore;
}

bool poseChanged(const Pose& a, const Pose& b) {
    return a.mirrored != b.mirrored ||
        std::abs(a.x - b.x) > 1e-6 ||
        std::abs(a.y - b.y) > 1e-6 ||
        std::abs(a.angleRadians - b.angleRadians) > 1e-8;
}

bool anySubsetPoseChanged(const std::vector<Pose>& poses, const std::vector<Pose>& reference, const std::vector<size_t>& subset) {
    for (size_t index : subset) {
        if (index < poses.size() && index < reference.size() && poseChanged(poses[index], reference[index])) {
            return true;
        }
    }
    return false;
}

} // namespace

LayoutState DestroyRebuild::improve(
    const Document& document,
    const EngineSettings& settings,
    LayoutState current,
    const std::atomic_bool& stopRequested,
    SolverStats* stats) const {
    if (settings.performanceProfile == PerformanceProfile::Fast || document.parts.empty() || current.poses.size() < document.parts.size()) {
        return current;
    }

    LayoutScore scorer;
    PenaltySystem penalties;
    current = scorer.evaluate(document, settings, current.poses, &penalties);
    if (!current.valid()) {
        return current;
    }

    LayoutState best = current;
    const auto started = Clock::now();
    const double budget = settings.timeLimitSeconds > 0.0
        ? std::min(12.0, std::max(4.0, settings.timeLimitSeconds * 0.22))
        : (settings.performanceProfile == PerformanceProfile::Maximum ? 14.0 : 6.0);
    auto expired = [&]() {
        return std::chrono::duration<double>(Clock::now() - started).count() >= budget;
    };

    AdaptiveAcceptance acceptance(settings);
    const int maxAttempts = settings.performanceProfile == PerformanceProfile::Maximum ? 8 : 4;
    for (int attempt = 0; attempt < maxAttempts && !stopRequested.load() && !expired(); ++attempt) {
        const EmptySpaceMap map = EmptySpaceAnalyzer{}.analyze(document, settings, current);
        std::vector<std::vector<size_t>> subsets = buildSubsets(document, current, map, settings);
        if (subsets.empty()) {
            break;
        }
        const std::vector<size_t>& subset = subsets[static_cast<size_t>(attempt) % subsets.size()];
        if (subset.size() < 4) {
            continue;
        }
        if (stats) {
            ++stats->destroyAttempts;
            stats->destroySubsetTotal += subset.size();
            stats->averageSubsetSize = static_cast<double>(stats->destroySubsetTotal) / static_cast<double>(std::max<size_t>(1, stats->destroyAttempts));
        }

        std::vector<size_t> order = subset;
        if (attempt % 3 == 0) {
            std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
                return partAreaScore(document.parts[a]) < partAreaScore(document.parts[b]);
            });
        } else if (attempt % 3 == 1) {
            std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
                return fitScore(a, document, current, map) > fitScore(b, document, current, map);
            });
        } else {
            std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
                const size_t ha = (a * 1103515245u + settings.randomSeed + static_cast<unsigned int>(attempt * 97)) & 0x7fffffffu;
                const size_t hb = (b * 1103515245u + settings.randomSeed + static_cast<unsigned int>(attempt * 97)) & 0x7fffffffu;
                return ha < hb;
            });
        }

        BeamNode root;
        root.poses = current.poses;
        root.rank = nodeRank(document, settings, root.poses, 0.0);
        std::vector<BeamNode> beam{std::move(root)};
        const size_t beamWidth = settings.performanceProfile == PerformanceProfile::Maximum ? 8u : 4u;
        const size_t leafLimit = settings.performanceProfile == PerformanceProfile::Maximum ? 20u : 12u;
        bool failed = false;
        const std::vector<Vec2> anchors = regionAnchors(map, current, document, subset);
        LayoutState bestPartial;
        bool hasPartial = false;
        auto considerPartial = [&](const BeamNode& node) {
            if (!anySubsetPoseChanged(node.poses, current.poses, subset)) {
                return;
            }
            LayoutState partial = scorer.evaluate(document, settings, node.poses, &penalties);
            if (!partial.valid()) {
                return;
            }
            if (stats) {
                ++stats->beamValidLeaves;
            }
            if (!hasPartial || partial.totalScore < bestPartial.totalScore) {
                bestPartial = std::move(partial);
                hasPartial = true;
            }
        };

        for (size_t partIndex : order) {
            if (stopRequested.load() || expired()) {
                failed = true;
                break;
            }
            std::vector<BeamNode> expanded;
            for (const BeamNode& node : beam) {
                AnalyticContactRequest request;
                request.movingPart = partIndex;
                request.fixedParts = fixedPartsFor(document, subset, node.placed);
                request.regionAnchors = anchors;
                request.angles = angleSamples(settings, current.poses[partIndex]);
                request.mirrors = mirrorSamples(settings, current.poses[partIndex]);
                request.ownerLimit = settings.performanceProfile == PerformanceProfile::Maximum ? 36u : 18u;
                request.perOwnerPointLimit = settings.performanceProfile == PerformanceProfile::Maximum ? 8u : 5u;
                request.candidateLimit = settings.performanceProfile == PerformanceProfile::Maximum ? 140u : 70u;
                AnalyticContactStats analyticStats;
                const std::vector<AnalyticContactCandidate> candidates =
                    AnalyticContactCandidateGenerator{}.generate(document, settings, node.poses, request, &analyticStats);
                std::vector<AnalyticContactCandidate> candidateList = candidates;
                if (partIndex < current.poses.size()) {
                    AnalyticContactCandidate keep;
                    keep.pose = current.poses[partIndex];
                    keep.kind = AnalyticContactKind::RegionAnchor;
                    keep.priority = -1000.0;
                    candidateList.insert(candidateList.begin(), keep);
                }
                if (stats) {
                    stats->analyticCandidatesGenerated += analyticStats.generated;
                    stats->analyticCandidatesValid += analyticStats.valid;
                    stats->contactCandidatesRejectedCollision += analyticStats.rejectedCollision;
                    stats->contactCandidatesRejectedClearance += analyticStats.rejectedClearance;
                    stats->contactCandidatesRejectedSheet += analyticStats.rejectedSheet;
                }
                size_t used = 0;
                for (const AnalyticContactCandidate& candidate : candidateList) {
                    BeamNode next = node;
                    next.poses[partIndex] = candidate.pose;
                    next.placed.push_back(partIndex);
                    next.depth = node.depth + 1;
                    next.contactPriority = node.contactPriority + candidate.priority;
                    next.rank = nodeRank(document, settings, next.poses, next.contactPriority);
                    expanded.push_back(std::move(next));
                    if (stats) {
                        ++stats->beamNodesExpanded;
                    }
                    if (++used >= leafLimit) {
                        break;
                    }
                }
            }
            if (expanded.empty()) {
                failed = true;
                break;
            }
            std::stable_sort(expanded.begin(), expanded.end(), [](const BeamNode& a, const BeamNode& b) {
                if (std::abs(a.rank - b.rank) > 1e-9) {
                    return a.rank < b.rank;
                }
                return a.contactPriority > b.contactPriority;
            });
            if (expanded.size() > beamWidth) {
                expanded.resize(beamWidth);
            }
            for (const BeamNode& node : expanded) {
                considerPartial(node);
            }
            beam = std::move(expanded);
        }

        bool acceptedAttempt = false;
        if (!failed) {
            for (size_t leaf = 0; leaf < beam.size() && !stopRequested.load() && !expired(); ++leaf) {
                considerPartial(beam[leaf]);
            }
        }

        auto acceptCandidate = [&](LayoutState candidate, size_t candidateIndex) {
            if (!candidate.valid()) {
                return false;
            }
            if (betterLayout(candidate, best)) {
                best = candidate;
                current = std::move(candidate);
                if (stats) {
                    ++stats->destroyAccepted;
                    ++stats->destroyBestUpdates;
                    ++stats->bestUpdates;
                    ++stats->acceptedMoves;
                    ++stats->activeMoveAcceptedTotal;
                    ++stats->acceptedBetter;
                }
                return true;
            }

            const double emptyPotential = map.totalEmptyArea / std::max(1.0, document.sheet.width * document.sheet.height);
            AdaptiveAcceptanceContext context;
            context.currentScore = current.totalScore;
            context.candidateScore = candidate.totalScore;
            context.bestScore = best.totalScore;
            context.emptySpacePotential = emptyPotential;
            context.contactPotential = candidate.contactReward / std::max(1.0, static_cast<double>(document.parts.size()));
            context.destroyRebuildMove = true;
            context.iteration = attempt;
            context.maxIterations = maxAttempts;
            context.seed = settings.randomSeed == 0u ? 1u : settings.randomSeed;
            context.candidateIndex = candidateIndex;
            const AdaptiveAcceptanceDecision decision = acceptance.decide(context);
            if (decision.accepted) {
                current = std::move(candidate);
                if (stats) {
                    ++stats->destroyAccepted;
                    ++stats->destroyTemporaryAccepted;
                    ++stats->acceptedTemporary;
                    ++stats->acceptedWorseMoves;
                    ++stats->activeMoveAcceptedTotal;
                }
                return true;
            }
            if (stats) {
                ++stats->rejectedByAcceptance;
            }
            return false;
        };

        if (hasPartial) {
            acceptedAttempt = acceptCandidate(std::move(bestPartial), static_cast<size_t>(attempt));
        }
        if (acceptedAttempt) {
            continue;
        }
    }
    return best.valid() ? best : current;
}

} // namespace nest
