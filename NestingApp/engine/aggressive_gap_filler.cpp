#include "engine/aggressive_gap_filler.h"

#include "engine/empty_space_map.h"
#include "engine/layout_score.h"
#include "engine/pose_sampler.h"
#include "engine/slide_to_contact.h"
#include "geometry/transformed_shape.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <numeric>

namespace nest {
namespace {

double footprint(const Part& part) {
    if (part.area > 0.0) {
        return part.area;
    }
    return part.localBounds.area();
}

std::vector<size_t> candidateParts(const Document& document, const LayoutState& state) {
    std::vector<size_t> order(document.parts.size());
    std::iota(order.begin(), order.end(), 0);
    AABB used;
    for (size_t i = 0; i < document.parts.size() && i < state.poses.size(); ++i) {
        used.include(transformedBounds(document.parts[i], state.poses[i]));
    }
    std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        const AABB boxA = transformedBounds(document.parts[a], state.poses[a]);
        const AABB boxB = transformedBounds(document.parts[b], state.poses[b]);
        const bool boundaryA = std::abs(boxA.min.x - used.min.x) <= 2.0 ||
            std::abs(boxA.max.x - used.max.x) <= 2.0 ||
            std::abs(boxA.min.y - used.min.y) <= 2.0 ||
            std::abs(boxA.max.y - used.max.y) <= 2.0;
        const bool boundaryB = std::abs(boxB.min.x - used.min.x) <= 2.0 ||
            std::abs(boxB.max.x - used.max.x) <= 2.0 ||
            std::abs(boxB.min.y - used.min.y) <= 2.0 ||
            std::abs(boxB.max.y - used.max.y) <= 2.0;
        if (boundaryA != boundaryB) {
            return boundaryA;
        }
        return footprint(document.parts[a]) < footprint(document.parts[b]);
    });
    return order;
}

Pose centeredPose(const Part& part, const Pose& orientation, Vec2 center) {
    const AABB bounds = transformedBounds(part, orientation);
    Pose pose = orientation;
    pose.x = center.x - bounds.center().x;
    pose.y = center.y - bounds.center().y;
    return pose;
}

std::vector<Vec2> regionAnchors(const EmptyRegion& region) {
    const double insetX = region.bounds.width() * 0.20;
    const double insetY = region.bounds.height() * 0.20;
    return {
        region.center,
        {region.bounds.min.x + insetX, region.bounds.min.y + insetY},
        {region.bounds.max.x - insetX, region.bounds.min.y + insetY},
        {region.bounds.min.x + insetX, region.bounds.max.y - insetY},
        {region.bounds.max.x - insetX, region.bounds.max.y - insetY},
        {region.bounds.center().x, region.bounds.min.y + insetY},
        {region.bounds.center().x, region.bounds.max.y - insetY},
        {region.bounds.min.x + insetX, region.bounds.center().y},
        {region.bounds.max.x - insetX, region.bounds.center().y}
    };
}

std::array<Vec2, 8> slideDirections(Vec2 from, Vec2 to) {
    Vec2 toward = to - from;
    const double len = std::max(1e-9, toward.length());
    toward.x /= len;
    toward.y /= len;
    return {
        toward,
        Vec2{-1.0, 0.0},
        Vec2{0.0, -1.0},
        Vec2{1.0, 0.0},
        Vec2{0.0, 1.0},
        Vec2{-0.70710678, -0.70710678},
        Vec2{0.70710678, -0.70710678},
        Vec2{-0.70710678, 0.70710678}
    };
}

} // namespace

LayoutState AggressiveGapFiller::improve(
    const Document& document,
    const EngineSettings& settings,
    LayoutState state,
    const std::atomic_bool& stopRequested,
    SolverStats* stats) const {
    if (!state.valid() || document.parts.empty() || settings.performanceProfile == PerformanceProfile::Fast) {
        return state;
    }

    LayoutScore scorer;
    PenaltySystem penalties;
    EmptySpaceAnalyzer analyzer;
    SlideToContact slider;
    const auto started = std::chrono::steady_clock::now();
    const double budgetSeconds = settings.timeLimitSeconds > 0.0
        ? std::min(8.0, std::max(2.0, settings.timeLimitSeconds * 0.22))
        : (settings.performanceProfile == PerformanceProfile::Maximum ? 10.0 : 4.0);
    auto budgetExpired = [&]() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() >= budgetSeconds;
    };
    const int passLimit = settings.performanceProfile == PerformanceProfile::Maximum ? 6 : 3;
    const size_t partLimit = document.parts.size() > 250 ? 48u : 60u;
    const size_t regionLimit = document.parts.size() > 250 ? 12u : 14u;

    for (int pass = 0; pass < passLimit && !stopRequested.load() && !budgetExpired(); ++pass) {
        EmptySpaceMap map = analyzer.analyze(document, settings, state);
        if (stats) {
            stats->emptySpaceArea = map.totalEmptyArea;
            stats->largestEmptyRegionArea = map.largestRegionArea;
            const double avgSmall = std::max(1.0, document.totalPartArea() / std::max<size_t>(1, document.parts.size()) * 0.35);
            stats->fillableGapCount = map.fillableRegionCount(avgSmall);
            stats->contactCount = static_cast<size_t>(std::llround(std::max(0.0, state.contactReward)));
        }
        if (map.regions.empty()) {
            break;
        }

        const std::vector<size_t> parts = candidateParts(document, state);
        bool acceptedInPass = false;
        const size_t maxParts = std::min(partLimit, parts.size());
        for (size_t pi = 0; pi < maxParts && !stopRequested.load() && !budgetExpired(); ++pi) {
            const size_t partIndex = parts[pi];
            if (partIndex >= state.poses.size()) {
                continue;
            }
            const Part& part = document.parts[partIndex];
            const double partArea = std::max(1.0, footprint(part));
            PoseSampler sampler;
            std::vector<double> angles{state.poses[partIndex].angleRadians};
            if (settings.allowRotation && settings.rotationMode != RotationMode::None) {
                const std::vector<double> coarse = sampler.coarseRotationSamples(settings);
                for (double angle : coarse) {
                    if (angles.size() >= 4) {
                        break;
                    }
                    if (std::none_of(angles.begin(), angles.end(), [&](double existing) { return std::abs(existing - angle) < 1e-9; })) {
                        angles.push_back(angle);
                    }
                }
            }
            std::vector<bool> mirrors{state.poses[partIndex].mirrored};
            if (settings.allowMirroring) {
                mirrors.push_back(!state.poses[partIndex].mirrored);
            }

            LayoutState bestCandidate;
            bool found = false;
            bool bestUsedSlide = false;
            for (size_t ri = 0; ri < map.regions.size() && ri < regionLimit && !stopRequested.load() && !budgetExpired(); ++ri) {
                const EmptyRegion& region = map.regions[ri];
                if (region.area < partArea * 0.35) {
                    continue;
                }
                const std::vector<Vec2> anchors = regionAnchors(region);
                for (double angle : angles) {
                    for (bool mirrored : mirrors) {
                        Pose orientation;
                        orientation.angleRadians = angle;
                        orientation.mirrored = mirrored;
                        const AABB oriented = transformedBounds(part, orientation);
                        if (oriented.width() > region.bounds.width() * 1.35 ||
                            oriented.height() > region.bounds.height() * 1.35) {
                            continue;
                        }
                        for (Vec2 anchor : anchors) {
                            Pose candidatePose = centeredPose(part, orientation, anchor);
                            std::vector<Pose> poses = state.poses;
                            poses[partIndex] = candidatePose;
                            LayoutState candidate = scorer.evaluate(document, settings, poses, &penalties);
                            if (!candidate.valid()) {
                                continue;
                            }
                            bool candidateUsedSlide = false;
                            const std::array<Vec2, 8> directions = slideDirections(transformedBounds(part, state.poses[partIndex]).center(), anchor);
                            for (Vec2 direction : directions) {
                                SlideToContactResult slide = slider.slide(document, settings, candidate, partIndex, candidatePose, direction, std::max(region.bounds.width(), region.bounds.height()));
                                if (slide.moved) {
                                    std::vector<Pose> slidePoses = state.poses;
                                    slidePoses[partIndex] = slide.pose;
                                    LayoutState slid = scorer.evaluate(document, settings, slidePoses, &penalties);
                                    if (slid.valid() && slid.totalScore + 1e-9 < candidate.totalScore) {
                                        candidate = std::move(slid);
                                        candidateUsedSlide = true;
                                    }
                                }
                            }
                            const double currentArea = std::max(1.0, state.usedWidth * state.usedHeight);
                            const double candidateArea = std::max(1.0, candidate.usedWidth * candidate.usedHeight);
                            const bool improvesPacking =
                                candidate.utilization > state.utilization + 1e-6 ||
                                candidateArea + 1e-6 < currentArea;
                            if (candidate.totalScore + 1e-9 < state.totalScore &&
                                improvesPacking &&
                                (!found || candidate.totalScore < bestCandidate.totalScore)) {
                                bestCandidate = std::move(candidate);
                                found = true;
                                bestUsedSlide = candidateUsedSlide;
                            }
                        }
                    }
                }
            }
            if (found) {
                state = std::move(bestCandidate);
                acceptedInPass = true;
                if (stats) {
                    ++stats->aggressiveGapAccepted;
                    ++stats->acceptedMoves;
                    ++stats->bestUpdates;
                    if (bestUsedSlide) {
                        ++stats->slideToContactAccepted;
                    }
                }
            }
        }
        if (!acceptedInPass) {
            break;
        }
    }

    return state;
}

} // namespace nest
