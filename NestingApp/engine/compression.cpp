#include "engine/compression.h"

#include "engine/layout_eval_cache.h"
#include "engine/layout_score.h"
#include "geometry/clearance.h"
#include "geometry/collision.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <numeric>

namespace nest {
namespace {

bool candidateIsValid(const Document& document, const std::vector<Pose>& poses, size_t movingIndex, const Pose& candidate, const EngineSettings& settings) {
    if (!partRespectsSheetClearance(document.parts[movingIndex], candidate, document.sheet, settings.margin, settings.collisionTolerance)) {
        return false;
    }
    for (size_t i = 0; i < document.parts.size(); ++i) {
        if (i == movingIndex || i >= poses.size()) {
            continue;
        }
        if (partsCollide(document.parts[movingIndex], candidate, document.parts[i], poses[i], settings.collisionTolerance)) {
            return false;
        }
        if (!partsRespectClearance(document.parts[movingIndex], candidate, document.parts[i], poses[i], settings.partSpacing, settings.collisionTolerance)) {
            return false;
        }
    }
    return true;
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

Vec2 nearestUserAnchor(const Document& document, const Vec2& center) {
    const auto& anchors = document.sheet.getUserPlacementPoints();
    Vec2 best = anchors.empty() ? Vec2{} : anchors.front();
    double bestDistance = anchors.empty() ? 0.0 : distance(center, best);
    for (const Vec2& anchor : anchors) {
        const double d = distance(center, anchor);
        if (d < bestDistance) {
            bestDistance = d;
            best = anchor;
        }
    }
    return best;
}

int signToward(double from, double to) {
    if (std::abs(to - from) < 1e-6) {
        return 0;
    }
    return to > from ? 1 : -1;
}

} // namespace

void Compression::compressLeftUp(const Document& document, const EngineSettings& settings, std::vector<Pose>& poses) const {
    const double step = std::max(1.0, settings.partSpacing * 0.5);
    for (int pass = 0; pass < 4; ++pass) {
        for (size_t i = 0; i < poses.size() && i < document.parts.size(); ++i) {
            for (int axis = 0; axis < 2; ++axis) {
                bool moved = true;
                while (moved) {
                    moved = false;
                    Pose candidate = poses[i];
                    if (axis == 0) {
                        candidate.x -= step;
                    } else {
                        candidate.y -= step;
                    }
                    if (candidateIsValid(document, poses, i, candidate, settings)) {
                        poses[i] = candidate;
                        moved = true;
                    }
                }
            }
        }
    }
}

void Compression::compressByStrategy(const Document& document, const EngineSettings& settings, std::vector<Pose>& poses) const {
    if (settings.placementStrategy == PlacementStrategy::BottomLeft ||
        settings.placementStrategy == PlacementStrategy::LeftToRight ||
        settings.placementStrategy == PlacementStrategy::BottomToTop) {
        compressLeftUp(document, settings, poses);
        return;
    }

    const double step = std::max(1.0, settings.partSpacing * 0.5);
    const Vec2 sheetCenter{
        document.sheet.origin.x + document.sheet.width * 0.5,
        document.sheet.origin.y + document.sheet.height * 0.5
    };

    for (int pass = 0; pass < 4; ++pass) {
        for (size_t i = 0; i < poses.size() && i < document.parts.size(); ++i) {
            for (int axis = 0; axis < 2; ++axis) {
                bool moved = true;
                const size_t maxMoves = static_cast<size_t>(std::max(1.0, (document.sheet.width + document.sheet.height) / step)) + 4;
                size_t moveCount = 0;
                while (moved && moveCount < maxMoves) {
                    moved = false;
                    Pose candidate = poses[i];
                    int sign = axis == 0
                        ? horizontalCompressionSign(settings.placementStrategy)
                        : verticalCompressionSign(settings.placementStrategy);
                    bool targetDriven = false;
                    double currentCoordinate = 0.0;
                    double targetCoordinate = 0.0;

                    if (settings.placementStrategy == PlacementStrategy::CenterOut ||
                        settings.placementStrategy == PlacementStrategy::OutsideIn ||
                        settings.placementStrategy == PlacementStrategy::UserPoints) {
                        const AABB box = transformedBounds(document.parts[i], poses[i]);
                        Vec2 target = sheetCenter;
                        if (settings.placementStrategy == PlacementStrategy::UserPoints && !document.sheet.getUserPlacementPoints().empty()) {
                            target = nearestUserAnchor(document, box.center());
                        }
                        currentCoordinate = axis == 0 ? box.center().x : box.center().y;
                        targetCoordinate = axis == 0 ? target.x : target.y;
                        sign = signToward(currentCoordinate, targetCoordinate);
                        targetDriven = true;
                    }

                    if (sign == 0) {
                        break;
                    }
                    if (targetDriven) {
                        const double currentDistance = std::abs(targetCoordinate - currentCoordinate);
                        const double nextDistance = std::abs(targetCoordinate - (currentCoordinate + static_cast<double>(sign) * step));
                        if (currentDistance <= step * 0.5 || nextDistance >= currentDistance - 1e-9) {
                            break;
                        }
                    }
                    if (axis == 0) {
                        candidate.x += static_cast<double>(sign) * step;
                    } else {
                        candidate.y += static_cast<double>(sign) * step;
                    }
                    if (candidateIsValid(document, poses, i, candidate, settings)) {
                        poses[i] = candidate;
                        moved = true;
                        ++moveCount;
                    }
                }
            }
        }
    }
}

LayoutState Compression::compressByScore(
    const Document& document,
    const EngineSettings& settings,
    LayoutState state,
    const PenaltySystem& penalties,
    SolverStats* stats,
    const std::atomic_bool* stopRequested) const {
    auto shouldStop = [&]() {
        return stopRequested && stopRequested->load();
    };

    LayoutScore scorer;
    state = scorer.evaluate(document, settings, state.poses, &penalties);
    if (shouldStop()) {
        return state;
    }
    const bool largeLayout = document.parts.size() > 60;
    const double initialStep = largeLayout
        ? std::max(4.0, std::min(32.0, std::min(document.sheet.width, document.sheet.height) * 0.04))
        : std::max(8.0, std::min(document.sheet.width, document.sheet.height) * 0.10);
    const double minStep = 0.25;
    const Vec2 sheetCenter{document.sheet.origin.x + document.sheet.width * 0.5, document.sheet.origin.y + document.sheet.height * 0.5};

    auto directionFor = [&](const Part& part, const Pose& pose, int axis) {
        if (settings.placementStrategy == PlacementStrategy::CenterOut || settings.placementStrategy == PlacementStrategy::OutsideIn) {
            const AABB box = transformedBounds(part, pose);
            const double current = axis == 0 ? box.center().x : box.center().y;
            const double target = axis == 0 ? sheetCenter.x : sheetCenter.y;
            return signToward(current, target);
        }
        return axis == 0 ? horizontalCompressionSign(settings.placementStrategy) : verticalCompressionSign(settings.placementStrategy);
    };

    auto shiftedPose = [](Pose pose, int axis, int sign, double distance) {
        if (axis == 0) {
            pose.x += static_cast<double>(sign) * distance;
        } else {
            pose.y += static_cast<double>(sign) * distance;
        }
        return pose;
    };

    auto poseWithMinAt = [](const Part& part, const Pose& base, Vec2 anchor) {
        Pose local;
        local.angleRadians = base.angleRadians;
        local.mirrored = base.mirrored;
        const AABB bounds = transformedBounds(part, local);
        Pose pose = base;
        pose.x = anchor.x - bounds.min.x;
        pose.y = anchor.y - bounds.min.y;
        return pose;
    };

    auto shelfOrder = [&](int mode) {
        std::vector<size_t> order(state.poses.size());
        std::iota(order.begin(), order.end(), 0);
        if (mode == 0) {
            std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
                const AABB boxA = transformedBounds(document.parts[a], state.poses[a]);
                const AABB boxB = transformedBounds(document.parts[b], state.poses[b]);
                if (std::abs(boxA.height() - boxB.height()) > 1e-9) {
                    return boxA.height() > boxB.height();
                }
                return boxA.width() > boxB.width();
            });
        } else if (mode == 1) {
            std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
                return document.parts[a].area > document.parts[b].area;
            });
        } else {
            std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
                const AABB boxA = transformedBounds(document.parts[a], state.poses[a]);
                const AABB boxB = transformedBounds(document.parts[b], state.poses[b]);
                if (std::abs(boxA.min.y - boxB.min.y) > 1e-9) {
                    return boxA.min.y < boxB.min.y;
                }
                return boxA.min.x < boxB.min.x;
            });
        }
        return order;
    };

    auto shelfRepack = [&](LayoutState base) {
        if (base.poses.empty() || document.parts.empty()) {
            return base;
        }
        const double left = document.sheet.origin.x + settings.margin;
        const double bottom = document.sheet.origin.y + settings.margin;
        const double maxWidth = std::max(1.0, document.sheet.width - settings.margin * 2.0);
        const double currentWidth = std::max(1.0, base.usedWidth);
        const double fastFactors[] = {0.95, 0.90};
        const double balancedFactors[] = {0.95, 0.90, 0.85, 0.80};
        const double maximumFactors[] = {0.95, 0.90, 0.85, 0.80, 0.72, 0.64};
        const double* factors = settings.performanceProfile == PerformanceProfile::Fast ? fastFactors :
            settings.performanceProfile == PerformanceProfile::Maximum ? maximumFactors : balancedFactors;
        const size_t factorCount = settings.performanceProfile == PerformanceProfile::Fast ? sizeof(fastFactors) / sizeof(fastFactors[0]) :
            settings.performanceProfile == PerformanceProfile::Maximum ? sizeof(maximumFactors) / sizeof(maximumFactors[0]) : sizeof(balancedFactors) / sizeof(balancedFactors[0]);
        const int orderModes = settings.performanceProfile == PerformanceProfile::Fast ? 1 :
            settings.performanceProfile == PerformanceProfile::Maximum ? 3 : 2;
        LayoutState best = base;

        for (int mode = 0; mode < orderModes && !shouldStop(); ++mode) {
            const std::vector<size_t> order = shelfOrder(mode);
            for (size_t factorIndex = 0; factorIndex < factorCount && !shouldStop(); ++factorIndex) {
                const double factor = factors[factorIndex];
                const double targetWidth = std::max(32.0, std::min(maxWidth, currentWidth * factor));
                std::vector<Pose> poses = base.poses;
                double x = left;
                double y = bottom;
                double shelfHeight = 0.0;

                for (size_t index : order) {
                    if (shouldStop()) {
                        break;
                    }
                    if (index >= document.parts.size() || index >= poses.size()) {
                        continue;
                    }
                    Pose orientationOnly;
                    orientationOnly.angleRadians = poses[index].angleRadians;
                    orientationOnly.mirrored = poses[index].mirrored;
                    const AABB bounds = transformedBounds(document.parts[index], orientationOnly);
                    const double width = std::max(1.0, bounds.width());
                    const double height = std::max(1.0, bounds.height());
                    if (x > left && x + width > left + targetWidth) {
                        x = left;
                        y += shelfHeight + settings.partSpacing;
                        shelfHeight = 0.0;
                    }
                    poses[index] = poseWithMinAt(document.parts[index], poses[index], {x, y});
                    x += width + settings.partSpacing;
                    shelfHeight = std::max(shelfHeight, height);
                }

                if (shouldStop()) {
                    break;
                }
                LayoutState trial = scorer.evaluate(document, settings, poses, &penalties);
                if (trial.valid() && trial.totalScore + 1e-9 < best.totalScore) {
                    best = std::move(trial);
                }
            }
        }
        return best;
    };

    LayoutState shelf = shelfRepack(state);
    if (shelf.valid() && shelf.totalScore + 1e-9 < state.totalScore) {
        state = std::move(shelf);
        if (stats) {
            ++stats->acceptedMoves;
            ++stats->bestUpdates;
            ++stats->compactionAccepted;
        }
    }

    auto evaluateShift = [&](const LayoutState& base, const LayoutEvalCache& cache, size_t partIndex, int axis, int sign, double distance) {
        std::vector<Pose> trialPoses = base.poses;
        trialPoses[partIndex] = shiftedPose(trialPoses[partIndex], axis, sign, distance);
        const DeltaMove move{partIndex, base.poses[partIndex], trialPoses[partIndex]};
        const DeltaEvaluation delta = evaluateMoveDelta(document, settings, base, cache, move);
        LayoutState trial = base;
        trial.poses = std::move(trialPoses);
        trial.collisionCount = delta.collisionCount;
        trial.invalidPartCount = delta.invalidPartCount;
        trial.spacingPenalty = delta.spacingPenalty;
        trial.sheetPenalty = delta.sheetPenalty;
        trial.usedWidth = delta.usedWidth;
        trial.usedHeight = delta.usedHeight;
        trial.totalScore = delta.totalScore;
        const double usedArea = std::max(1.0, delta.usedWidth * delta.usedHeight);
        trial.utilization = std::max(0.0, std::min(1.0, document.totalPartArea() / usedArea));
        return trial;
    };

    auto stateUsesSamePoses = [](const LayoutState& a, const LayoutState& b) {
        if (a.poses.size() != b.poses.size()) {
            return false;
        }
        for (size_t i = 0; i < a.poses.size(); ++i) {
            if (std::abs(a.poses[i].x - b.poses[i].x) > 1e-9 ||
                std::abs(a.poses[i].y - b.poses[i].y) > 1e-9 ||
                std::abs(a.poses[i].angleRadians - b.poses[i].angleRadians) > 1e-9 ||
                a.poses[i].mirrored != b.poses[i].mirrored) {
                return false;
            }
        }
        return true;
    };

    auto acceptableCompaction = [](const LayoutState& before, const LayoutState& after) {
        if (!after.valid()) {
            return false;
        }
        if (after.totalScore + 1e-9 < before.totalScore) {
            return true;
        }
        if (after.totalScore <= before.totalScore + 1e-9 &&
            after.usedWidth <= before.usedWidth + 1e-9 &&
            after.usedHeight <= before.usedHeight + 1e-9) {
            return true;
        }
        return false;
    };

    auto pathIsSafe = [&](const LayoutState& base, const LayoutEvalCache& cache, size_t partIndex, int axis, int sign, double distance) {
        const double sampleStep = std::max(1.0, settings.partSpacing * 0.5);
        const int samples = std::max(1, static_cast<int>(std::ceil(distance / sampleStep)));
        for (int sample = 1; sample <= samples && !shouldStop(); ++sample) {
            const double d = distance * static_cast<double>(sample) / static_cast<double>(samples);
            const LayoutState trial = evaluateShift(base, cache, partIndex, axis, sign, d);
            if (!trial.valid()) {
                return false;
            }
        }
        return true;
    };

    auto bestRefinedShift = [&](const LayoutState& base, size_t partIndex, int axis, int sign) {
        double accepted = 0.0;
        double step = initialStep;
        LayoutState best = base;
        LayoutEvalCache cache;
        cache.rebuild(document, settings, base, &penalties);
        while (step >= minStep && !shouldStop()) {
            const double candidateDistance = accepted + step;
            LayoutState trial = evaluateShift(base, cache, partIndex, axis, sign, candidateDistance);
            if (acceptableCompaction(best, trial) && (largeLayout || pathIsSafe(base, cache, partIndex, axis, sign, candidateDistance))) {
                std::vector<Pose> verifiedPoses = base.poses;
                verifiedPoses[partIndex] = shiftedPose(verifiedPoses[partIndex], axis, sign, candidateDistance);
                LayoutState verified = scorer.evaluate(document, settings, verifiedPoses, &penalties);
                if (acceptableCompaction(best, verified) && !stateUsesSamePoses(best, verified)) {
                    accepted = candidateDistance;
                    best = std::move(verified);
                } else {
                    step *= 0.5;
                }
            } else {
                step *= 0.5;
            }
        }
        return best;
    };

    auto makeOrder = [&](const LayoutState& layout, int axis, int sign, int mode) {
        std::vector<size_t> order(layout.poses.size());
        std::iota(order.begin(), order.end(), 0);
        if (mode == 1 || mode == 2) {
            std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
                const AABB boxA = transformedBounds(document.parts[a], layout.poses[a]);
                const AABB boxB = transformedBounds(document.parts[b], layout.poses[b]);
                const double primaryA = axis == 0 ? (sign < 0 ? boxA.min.x : boxA.max.x) : (sign < 0 ? boxA.min.y : boxA.max.y);
                const double primaryB = axis == 0 ? (sign < 0 ? boxB.min.x : boxB.max.x) : (sign < 0 ? boxB.min.y : boxB.max.y);
                if (std::abs(primaryA - primaryB) > 1e-9) {
                    return mode == 1 ? primaryA < primaryB : primaryA > primaryB;
                }
                return document.parts[a].area > document.parts[b].area;
            });
        } else if (mode == 3) {
            std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
                return document.parts[a].localBounds.area() < document.parts[b].localBounds.area();
            });
        } else if (mode == 4) {
            // Deterministic shuffle-like order: avoids a random source while breaking row-order bias.
            std::sort(order.begin(), order.end(), [](size_t a, size_t b) {
                const size_t ha = (a * 1103515245u + 12345u) & 0x7fffffffu;
                const size_t hb = (b * 1103515245u + 12345u) & 0x7fffffffu;
                return ha < hb;
            });
        }
        if (largeLayout) {
            const size_t limit = settings.performanceProfile == PerformanceProfile::Maximum ? 36u :
                settings.performanceProfile == PerformanceProfile::Balanced ? 28u : 18u;
            if (order.size() > limit) {
                order.resize(limit);
            }
        }
        return order;
    };

    const int passes = largeLayout
        ? (settings.performanceProfile == PerformanceProfile::Maximum ? 2 : settings.performanceProfile == PerformanceProfile::Balanced ? 2 : 1)
        : (settings.performanceProfile == PerformanceProfile::Fast ? 4 : settings.performanceProfile == PerformanceProfile::Maximum ? 10 : 7);
    const int orderModeLimit = largeLayout ? 1 : 5;
    for (int pass = 0; pass < passes && !shouldStop(); ++pass) {
        bool changed = false;
        const int firstAxis = pass % 2;
        const int axes[] = {firstAxis, 1 - firstAxis};
        for (int axis : axes) {
            const int sampleSign = state.poses.empty() ? -1 : directionFor(document.parts.front(), state.poses.front(), axis);
            for (int orderMode = 0; orderMode < orderModeLimit && !shouldStop(); ++orderMode) {
                const std::vector<size_t> order = makeOrder(state, axis, sampleSign == 0 ? -1 : sampleSign, orderMode);
                for (size_t i : order) {
                    if (shouldStop()) {
                        break;
                    }
                    if (i >= document.parts.size()) {
                        continue;
                    }
                    const int sign = directionFor(document.parts[i], state.poses[i], axis);
                    if (sign == 0) {
                        continue;
                    }
                    LayoutState trial = bestRefinedShift(state, i, axis, sign);
                    if (trial.valid() && !stateUsesSamePoses(state, trial) && trial.totalScore <= state.totalScore + 1e-9) {
                        const bool improvedScore = trial.totalScore + 1e-9 < state.totalScore;
                        state = std::move(trial);
                        changed = true;
                        if (stats) {
                            ++stats->acceptedMoves;
                            if (improvedScore) {
                                ++stats->bestUpdates;
                            }
                            ++stats->compactionAccepted;
                        }
                    }
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
