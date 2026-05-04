#include "engine/layout_eval_cache.h"

#include "engine/broadphase.h"
#include "engine/layout_score_components.h"
#include "geometry/clearance.h"
#include "geometry/collision.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace nest {
namespace {

constexpr double kCollisionPenalty = 100000000.0;
constexpr double kSheetPenalty = 100000000.0;
constexpr double kSpacingPenalty = 1000000.0;
constexpr double kUsedAreaPenalty = 1.0;
constexpr double kUtilizationReward = 10000.0;
constexpr double kCompactnessReward = 0.02;
constexpr double kCavityReward = 2500.0;

double clearanceDeficit(double required, double actual) {
    if (!std::isfinite(actual)) {
        return 0.0;
    }
    return std::max(0.0, required - actual) / std::max(1.0, required);
}

Ring physicalSheetOuter(const Sheet& sheet) {
    if (sheet.hasCustomProfile() && !sheet.profile().outerContour.points.empty()) {
        return sheet.profile().outerContour;
    }
    return sheet.makeRectangularOuterContour();
}

double sheetFeatureDistance(const Part& part, const Pose& pose, const Sheet& sheet, double requiredMargin, double eps) {
    const TransformedPart transformed = transformPart(part, pose);
    double best = minimumPartToRingDistance(transformed, physicalSheetOuter(sheet), requiredMargin, eps).minDistance;
    for (const Ring& hole : sheet.profile().holes) {
        best = std::min(best, minimumPartToRingDistance(transformed, hole, requiredMargin, eps).minDistance);
    }
    for (const Ring& zone : sheet.profile().forbiddenZones) {
        best = std::min(best, minimumPartToRingDistance(transformed, zone, requiredMargin, eps).minDistance);
    }
    return best;
}

double pairWeight(const LayoutEvalCache& cache, size_t a, size_t b) {
    const double attemptWeight = cache.attemptPenalties() ? cache.attemptPenalties()->weight(a, b) : 1.0;
    const double globalWeight = cache.globalPenalties() ? cache.globalPenalties()->weight(a, b) : 1.0;
    return attemptWeight * (1.0 + std::max(0.0, cache.globalPenaltyWeight()) * std::max(0.0, globalWeight - 1.0));
}

PairScoreContribution evaluatePair(
    const Document& document,
    const EngineSettings& settings,
    const Pose& poseA,
    const Pose& poseB,
    size_t a,
    size_t b,
    const LayoutEvalCache& cache) {
    PairScoreContribution contribution;
    if (a >= document.parts.size() || b >= document.parts.size()) {
        return contribution;
    }

    const AABB boxA = transformedBounds(document.parts[a], poseA);
    const AABB boxB = transformedBounds(document.parts[b], poseB);
    const double weight = pairWeight(cache, a, b);
    if (partsCollide(document.parts[a], poseA, document.parts[b], poseB, settings.collisionTolerance)) {
        contribution.collisionCount = 1;
        contribution.overlapPenalty = weight * (1.0 + aabbOverlapArea(boxA, boxB));
        return contribution;
    }

    const ClearanceResult clearance = minimumBoundaryDistance(
        transformPart(document.parts[a], poseA, static_cast<int>(a)),
        transformPart(document.parts[b], poseB, static_cast<int>(b)),
        settings.partSpacing,
        settings.collisionTolerance);
    if (!clearance.valid) {
        contribution.spacingPenalty = weight * std::max(0.01, clearanceDeficit(settings.partSpacing, clearance.minDistance));
    }
    return contribution;
}

SheetScoreContribution evaluateSheet(const Document& document, const EngineSettings& settings, size_t index, const Pose& pose) {
    SheetScoreContribution contribution;
    if (index >= document.parts.size()) {
        return contribution;
    }

    const ClearanceSettings clearance{settings.partSpacing, settings.margin, settings.collisionTolerance};
    bool invalid = false;
    if (!isPartInsideSheet(document.parts[index], pose, document.sheet, settings.collisionTolerance) ||
        overlapsSheetHolesOrForbiddenZones(document.parts[index], pose, document.sheet, settings.collisionTolerance)) {
        invalid = true;
    }
    if (!partRespectsSheetMargin(document.parts[index], pose, document.sheet, clearance)) {
        invalid = true;
        const double minDistance = sheetFeatureDistance(document.parts[index], pose, document.sheet, clearance.sheetMargin, settings.collisionTolerance);
        contribution.sheetPenalty += std::max(0.01, clearanceDeficit(clearance.sheetMargin, minDistance));
    }
    if (invalid) {
        contribution.invalidPartCount = 1;
        contribution.sheetPenalty += 1.0;
    }
    return contribution;
}

bool touchesUsedBoundary(const AABB& partBounds, const AABB& usedBounds, double eps) {
    if (!partBounds.isValid() || !usedBounds.isValid()) {
        return true;
    }
    return std::abs(partBounds.min.x - usedBounds.min.x) <= eps ||
        std::abs(partBounds.min.y - usedBounds.min.y) <= eps ||
        std::abs(partBounds.max.x - usedBounds.max.x) <= eps ||
        std::abs(partBounds.max.y - usedBounds.max.y) <= eps;
}

double totalScore(
    const Document& document,
    int collisionCount,
    double overlapPenalty,
    int invalidPartCount,
    double sheetPenalty,
    double spacingPenalty,
    const AABB& usedBounds,
    double cavityReward,
    double& usedWidth,
    double& usedHeight) {
    usedWidth = usedBounds.width();
    usedHeight = usedBounds.height();
    const double usedArea = std::max(1.0, usedBounds.area());
    const double utilization = std::max(0.0, std::min(1.0, document.totalPartArea() / usedArea));
    const double compactness = document.totalPartArea() / usedArea;
    return
        static_cast<double>(collisionCount) * kCollisionPenalty +
        overlapPenalty * kCollisionPenalty * 0.00001 +
        static_cast<double>(invalidPartCount) * kSheetPenalty +
        sheetPenalty * kSheetPenalty +
        spacingPenalty * kSpacingPenalty +
        usedArea * kUsedAreaPenalty -
        utilization * kUtilizationReward -
        compactness * kCompactnessReward -
        cavityReward * kCavityReward;
}

} // namespace

LayoutEvalCache::PairKey LayoutEvalCache::makePairKey(size_t a, size_t b) {
    if (b < a) {
        std::swap(a, b);
    }
    return {a, b};
}

void LayoutEvalCache::insertBounds(size_t index, const AABB& bounds) {
    if (!bounds.isValid()) {
        return;
    }
    const int minX = static_cast<int>(std::floor(bounds.min.x / cellSize_));
    const int maxX = static_cast<int>(std::floor(bounds.max.x / cellSize_));
    const int minY = static_cast<int>(std::floor(bounds.min.y / cellSize_));
    const int maxY = static_cast<int>(std::floor(bounds.max.y / cellSize_));
    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            grid_[{x, y}].push_back(index);
        }
    }
}

void LayoutEvalCache::rebuildSpatialIndex(double spacing) {
    grid_.clear();
    for (size_t i = 0; i < partBounds_.size(); ++i) {
        insertBounds(i, partBounds_[i].expanded(spacing));
    }
}

void LayoutEvalCache::rebuild(
    const Document& document,
    const EngineSettings& settings,
    const LayoutState& state,
    const PenaltySystem* attemptPenalties,
    const PenaltySystem* globalPenalties,
    double globalPenaltyWeight) {
    attemptPenalties_ = attemptPenalties;
    globalPenalties_ = globalPenalties;
    globalPenaltyWeight_ = globalPenaltyWeight;

    const size_t count = std::min(document.parts.size(), state.poses.size());
    transformedParts_.clear();
    transformedParts_.reserve(count);
    partBounds_.clear();
    partBounds_.reserve(count);
    usedBounds_ = {};
    for (size_t i = 0; i < count; ++i) {
        transformedParts_.push_back(transformPart(document.parts[i], state.poses[i], static_cast<int>(i)));
        partBounds_.push_back(transformedParts_.back().bounds);
        usedBounds_.include(partBounds_.back());
    }

    cellSize_ = std::max(32.0, settings.partSpacing * 8.0 + 64.0);
    rebuildSpatialIndex(settings.partSpacing);

    collisionCount_ = 0;
    pairOverlapPenalty_ = 0.0;
    spacingPenalty_ = 0.0;
    pairContributions_.clear();
    pairsByPart_.assign(count, {});

    BroadPhase broad;
    const auto pairs = broad.findCandidatePairs(document.parts, state.poses, settings.partSpacing);
    for (const auto& [a, b] : pairs) {
        if (a >= count || b >= count) {
            continue;
        }
        PairScoreContribution contribution = evaluatePair(document, settings, state.poses[a], state.poses[b], a, b, *this);
        const PairKey key = makePairKey(a, b);
        pairContributions_[key] = contribution;
        pairsByPart_[a].push_back(b);
        pairsByPart_[b].push_back(a);
        collisionCount_ += contribution.collisionCount;
        pairOverlapPenalty_ += contribution.overlapPenalty;
        spacingPenalty_ += contribution.spacingPenalty;
    }

    invalidPartCount_ = 0;
    sheetPenalty_ = 0.0;
    sheetContributions_.assign(count, {});
    for (size_t i = 0; i < count; ++i) {
        sheetContributions_[i] = evaluateSheet(document, settings, i, state.poses[i]);
        invalidPartCount_ += sheetContributions_[i].invalidPartCount;
        sheetPenalty_ += sheetContributions_[i].sheetPenalty;
    }
}

void LayoutEvalCache::updateAfterAcceptedMove(
    const Document& document,
    const EngineSettings& settings,
    const LayoutState& state,
    const DeltaMove&,
    const PenaltySystem* attemptPenalties,
    const PenaltySystem* globalPenalties,
    double globalPenaltyWeight) {
    rebuild(document, settings, state, attemptPenalties, globalPenalties, globalPenaltyWeight);
}

std::vector<size_t> LayoutEvalCache::queryNeighbors(const AABB& bounds, size_t excludedPart) const {
    std::set<size_t> unique;
    if (!bounds.isValid()) {
        return {};
    }
    const int minX = static_cast<int>(std::floor(bounds.min.x / cellSize_));
    const int maxX = static_cast<int>(std::floor(bounds.max.x / cellSize_));
    const int minY = static_cast<int>(std::floor(bounds.min.y / cellSize_));
    const int maxY = static_cast<int>(std::floor(bounds.max.y / cellSize_));
    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            auto it = grid_.find({x, y});
            if (it == grid_.end()) {
                continue;
            }
            for (size_t index : it->second) {
                if (index != excludedPart) {
                    unique.insert(index);
                }
            }
        }
    }
    return {unique.begin(), unique.end()};
}

PairScoreContribution LayoutEvalCache::pairContribution(size_t a, size_t b) const {
    auto it = pairContributions_.find(makePairKey(a, b));
    return it == pairContributions_.end() ? PairScoreContribution{} : it->second;
}

const std::vector<size_t>& LayoutEvalCache::pairsForPart(size_t index) const {
    static const std::vector<size_t> empty;
    return index < pairsByPart_.size() ? pairsByPart_[index] : empty;
}

DeltaEvaluation evaluateMoveDelta(
    const Document& document,
    const EngineSettings& settings,
    const LayoutState& current,
    const LayoutEvalCache& cache,
    const DeltaMove& move) {
    DeltaEvaluation evaluation;
    if (move.partIndex >= document.parts.size() || move.partIndex >= current.poses.size() ||
        move.partIndex >= cache.partBounds().size()) {
        return evaluation;
    }

    int collisionCount = cache.collisionCount();
    double overlapPenalty = cache.pairOverlapPenalty();
    double spacingPenalty = cache.spacingPenalty();
    int invalidPartCount = cache.invalidPartCount();
    double sheetPenalty = cache.sheetPenalty();

    for (size_t other : cache.pairsForPart(move.partIndex)) {
        const PairScoreContribution old = cache.pairContribution(move.partIndex, other);
        collisionCount -= old.collisionCount;
        overlapPenalty -= old.overlapPenalty;
        spacingPenalty -= old.spacingPenalty;
    }

    const SheetScoreContribution oldSheet = cache.sheetContributions()[move.partIndex];
    invalidPartCount -= oldSheet.invalidPartCount;
    sheetPenalty -= oldSheet.sheetPenalty;

    const AABB newBounds = transformedBounds(document.parts[move.partIndex], move.newPose);
    std::vector<size_t> neighbors = cache.queryNeighbors(newBounds.expanded(settings.partSpacing), move.partIndex);
    for (size_t other : neighbors) {
        if (other >= current.poses.size()) {
            continue;
        }
        const PairScoreContribution contribution = move.partIndex < other
            ? evaluatePair(document, settings, move.newPose, current.poses[other], move.partIndex, other, cache)
            : evaluatePair(document, settings, current.poses[other], move.newPose, other, move.partIndex, cache);
        collisionCount += contribution.collisionCount;
        overlapPenalty += contribution.overlapPenalty;
        spacingPenalty += contribution.spacingPenalty;
    }

    const SheetScoreContribution newSheet = evaluateSheet(document, settings, move.partIndex, move.newPose);
    invalidPartCount += newSheet.invalidPartCount;
    sheetPenalty += newSheet.sheetPenalty;

    AABB used = cache.usedBounds();
    if (touchesUsedBoundary(cache.partBounds()[move.partIndex], cache.usedBounds(), settings.collisionTolerance)) {
        used = {};
        for (size_t i = 0; i < cache.partBounds().size(); ++i) {
            used.include(i == move.partIndex ? newBounds : cache.partBounds()[i]);
        }
    } else {
        used.include(newBounds);
    }

    evaluation.collisionCount = collisionCount;
    evaluation.invalidPartCount = invalidPartCount;
    evaluation.spacingPenalty = std::max(0.0, spacingPenalty);
    evaluation.sheetPenalty = std::max(0.0, sheetPenalty);
    std::vector<Pose> candidatePoses = current.poses;
    candidatePoses[move.partIndex] = move.newPose;
    const double cavityReward = cavityPlacementReward(document, candidatePoses);
    evaluation.totalScore = totalScore(document, collisionCount, std::max(0.0, overlapPenalty), invalidPartCount, evaluation.sheetPenalty, evaluation.spacingPenalty, used, cavityReward, evaluation.usedWidth, evaluation.usedHeight);
    evaluation.valid = collisionCount == 0 && invalidPartCount == 0 && evaluation.spacingPenalty <= 0.0 && evaluation.sheetPenalty <= 0.0;
    return evaluation;
}

} // namespace nest
