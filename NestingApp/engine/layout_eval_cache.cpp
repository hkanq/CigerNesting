#include "engine/layout_eval_cache.h"

#include "engine/broadphase.h"
#include "engine/layout_score_components.h"
#include "geometry/clearance.h"
#include "geometry/collision.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace nest {
namespace {

constexpr double kCollisionPenalty = 100000000.0;
constexpr double kSheetPenalty = 100000000.0;
constexpr double kSpacingPenalty = 1000000.0;
constexpr double kUsedAreaPenalty = 1.0;
constexpr double kUtilizationReward = 10000.0;
constexpr double kCompactnessReward = 0.02;
constexpr double kCavityReward = 75000.0;
constexpr double kContactReward = 450.0;

double clearanceDeficit(double required, double actual) {
    if (!std::isfinite(actual)) {
        return 0.0;
    }
    return std::max(0.0, required - actual) / std::max(1.0, required);
}

double aabbDistance(const AABB& a, const AABB& b) {
    if (!a.isValid() || !b.isValid()) {
        return std::numeric_limits<double>::max();
    }
    const double dx = std::max({0.0, b.min.x - a.max.x, a.min.x - b.max.x});
    const double dy = std::max({0.0, b.min.y - a.max.y, a.min.y - b.max.y});
    return std::sqrt(dx * dx + dy * dy);
}

bool samePoint(Vec2 a, Vec2 b, double eps) {
    return distance(a, b) <= eps;
}

size_t effectivePointCount(const std::vector<Vec2>& points, double eps) {
    if (points.empty()) {
        return 0;
    }
    return points.size() > 2 && samePoint(points.front(), points.back(), eps) ? points.size() - 1 : points.size();
}

template <typename Callback>
void forEachSegment(const std::vector<Vec2>& points, double eps, Callback callback) {
    const size_t count = effectivePointCount(points, eps);
    if (count < 2) {
        return;
    }
    for (size_t i = 0; i < count; ++i) {
        const Vec2 a = points[i];
        const Vec2 b = points[(i + 1) % count];
        if (!samePoint(a, b, eps)) {
            callback(a, b);
        }
    }
}

bool anyRingSegmentsIntersect(const std::vector<Vec2>& a, const std::vector<Vec2>& b, double eps) {
    bool intersects = false;
    forEachSegment(a, eps, [&](Vec2 a0, Vec2 a1) {
        if (intersects) {
            return;
        }
        forEachSegment(b, eps, [&](Vec2 b0, Vec2 b1) {
            if (!intersects && segmentsIntersect(a0, a1, b0, b1, eps)) {
                intersects = true;
            }
        });
    });
    return intersects;
}

bool anySolidSampleInsideOther(const TransformedPart& source, const TransformedPart& other, double eps) {
    for (const TransformedRing& ring : source.rings) {
        if (ring.isHole || ring.points.empty()) {
            continue;
        }
        const size_t count = effectivePointCount(ring.points, eps);
        for (size_t i = 0; i < count; ++i) {
            const Vec2 a = ring.points[i];
            const Vec2 b = ring.points[(i + 1) % count];
            if (pointInSolidArea(other, a, eps)) {
                return true;
            }
            const Vec2 mid = (a + b) * 0.5;
            if (pointInSolidArea(other, mid, eps)) {
                return true;
            }
        }
    }
    return false;
}

bool transformedPartsCollide(const TransformedPart& partA, const TransformedPart& partB, double eps) {
    if (!partA.bounds.expanded(eps).overlaps(partB.bounds.expanded(eps))) {
        return false;
    }

    for (const TransformedRing& ringA : partA.rings) {
        if (ringA.points.size() < 2) {
            continue;
        }
        for (const TransformedRing& ringB : partB.rings) {
            if (ringB.points.size() < 2) {
                continue;
            }
            if (ringA.bounds.expanded(eps).overlaps(ringB.bounds.expanded(eps)) &&
                anyRingSegmentsIntersect(ringA.points, ringB.points, eps)) {
                return true;
            }
        }
    }

    return anySolidSampleInsideOther(partA, partB, eps) || anySolidSampleInsideOther(partB, partA, eps);
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
    const TransformedPart& transformedA,
    const TransformedPart& transformedB,
    size_t a,
    size_t b,
    const LayoutEvalCache& cache) {
    PairScoreContribution contribution;
    if (a >= document.parts.size() || b >= document.parts.size()) {
        return contribution;
    }

    const AABB boxA = transformedA.bounds;
    const AABB boxB = transformedB.bounds;
    const double weight = pairWeight(cache, a, b);
    if (transformedPartsCollide(transformedA, transformedB, settings.collisionTolerance)) {
        contribution.collisionCount = 1;
        contribution.overlapPenalty = weight * (1.0 + aabbOverlapArea(boxA, boxB));
        contribution.clearanceValid = false;
        contribution.clearanceDistance = 0.0;
        return contribution;
    }

    const double required = std::max(0.0, settings.partSpacing);
    const double broadDistance = aabbDistance(boxA, boxB);
    const double marginBuffer = std::max(settings.collisionTolerance * 4.0, required * 0.05);
    if (broadDistance >= required + marginBuffer) {
        contribution.clearanceValid = true;
        contribution.clearanceDistance = broadDistance;
        contribution.exactClearanceEvaluated = false;
        return contribution;
    }

    const ClearanceResult clearance = minimumBoundaryDistance(transformedA, transformedB, required, settings.collisionTolerance);
    contribution.clearanceValid = clearance.valid;
    contribution.clearanceDistance = clearance.minDistance;
    contribution.exactClearanceEvaluated = true;
    if (!clearance.valid) {
        contribution.spacingPenalty = weight * std::max(0.01, clearanceDeficit(required, clearance.minDistance));
    } else {
        const double contactWindow = std::max(0.05, settings.collisionTolerance * 10.0);
        if (std::abs(clearance.minDistance - required) <= contactWindow) {
            contribution.contactReward = weight;
        }
    }
    return contribution;
}

PairScoreContribution evaluatePair(
    const Document& document,
    const EngineSettings& settings,
    const Pose& poseA,
    const Pose& poseB,
    size_t a,
    size_t b,
    const LayoutEvalCache& cache) {
    return evaluatePair(
        document,
        settings,
        transformPart(document.parts[a], poseA, static_cast<int>(a)),
        transformPart(document.parts[b], poseB, static_cast<int>(b)),
        a,
        b,
        cache);
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
    double contactReward,
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
        cavityReward * kCavityReward -
        contactReward * kContactReward;
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
    if (index >= cellsByPart_.size()) {
        cellsByPart_.resize(index + 1);
    }
    const int minX = static_cast<int>(std::floor(bounds.min.x / cellSize_));
    const int maxX = static_cast<int>(std::floor(bounds.max.x / cellSize_));
    const int minY = static_cast<int>(std::floor(bounds.min.y / cellSize_));
    const int maxY = static_cast<int>(std::floor(bounds.max.y / cellSize_));
    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            const CellKey key{x, y};
            grid_[key].push_back(index);
            cellsByPart_[index].push_back(key);
        }
    }
}

void LayoutEvalCache::rebuildSpatialIndex(double spacing) {
    grid_.clear();
    cellsByPart_.assign(partBounds_.size(), {});
    for (size_t i = 0; i < partBounds_.size(); ++i) {
        insertBounds(i, partBounds_[i].expanded(spacing));
    }
}

void LayoutEvalCache::removeBounds(size_t index) {
    if (index >= cellsByPart_.size()) {
        return;
    }
    for (const CellKey& key : cellsByPart_[index]) {
        auto it = grid_.find(key);
        if (it == grid_.end()) {
            continue;
        }
        std::vector<size_t>& bucket = it->second;
        bucket.erase(std::remove(bucket.begin(), bucket.end(), index), bucket.end());
        if (bucket.empty()) {
            grid_.erase(it);
        }
    }
    cellsByPart_[index].clear();
}

void LayoutEvalCache::erasePairContribution(size_t a, size_t b) {
    const PairKey key = makePairKey(a, b);
    auto it = pairContributions_.find(key);
    if (it == pairContributions_.end()) {
        return;
    }
    collisionCount_ -= it->second.collisionCount;
    pairOverlapPenalty_ -= it->second.overlapPenalty;
    spacingPenalty_ -= it->second.spacingPenalty;
    contactReward_ -= it->second.contactReward;
    pairContributions_.erase(it);
    if (a < pairsByPart_.size()) {
        std::vector<size_t>& pairs = pairsByPart_[a];
        pairs.erase(std::remove(pairs.begin(), pairs.end(), b), pairs.end());
    }
    if (b < pairsByPart_.size()) {
        std::vector<size_t>& pairs = pairsByPart_[b];
        pairs.erase(std::remove(pairs.begin(), pairs.end(), a), pairs.end());
    }
}

void LayoutEvalCache::storePairContribution(size_t a, size_t b, const PairScoreContribution& contribution) {
    if (a == b) {
        return;
    }
    erasePairContribution(a, b);
    pairContributions_[makePairKey(a, b)] = contribution;
    if (a < pairsByPart_.size()) {
        pairsByPart_[a].push_back(b);
    }
    if (b < pairsByPart_.size()) {
        pairsByPart_[b].push_back(a);
    }
    collisionCount_ += contribution.collisionCount;
    pairOverlapPenalty_ += contribution.overlapPenalty;
    spacingPenalty_ += contribution.spacingPenalty;
    contactReward_ += contribution.contactReward;
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
    contactReward_ = 0.0;
    pairContributions_.clear();
    pairsByPart_.assign(count, {});

    BroadPhase broad;
    const auto pairs = broad.findCandidatePairs(document.parts, state.poses, settings.partSpacing);
    for (const auto& [a, b] : pairs) {
        if (a >= count || b >= count) {
            continue;
        }
        PairScoreContribution contribution = evaluatePair(document, settings, transformedParts_[a], transformedParts_[b], a, b, *this);
        storePairContribution(a, b, contribution);
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
    const DeltaMove& move,
    const PenaltySystem* attemptPenalties,
    const PenaltySystem* globalPenalties,
    double globalPenaltyWeight) {
    MultiDeltaMove multi;
    multi.partIndices.push_back(move.partIndex);
    multi.oldPoses.push_back(move.oldPose);
    multi.newPoses.push_back(move.newPose);
    updateAfterAcceptedMultiMove(document, settings, state, multi, attemptPenalties, globalPenalties, globalPenaltyWeight);
}

void LayoutEvalCache::updateAfterAcceptedMultiMove(
    const Document& document,
    const EngineSettings& settings,
    const LayoutState& state,
    const MultiDeltaMove& move,
    const PenaltySystem* attemptPenalties,
    const PenaltySystem* globalPenalties,
    double globalPenaltyWeight) {
    attemptPenalties_ = attemptPenalties;
    globalPenalties_ = globalPenalties;
    globalPenaltyWeight_ = globalPenaltyWeight;

    const size_t count = std::min(document.parts.size(), state.poses.size());
    if (count == 0 || transformedParts_.size() != count || partBounds_.size() != count ||
        sheetContributions_.size() != count || pairsByPart_.size() != count) {
        rebuild(document, settings, state, attemptPenalties, globalPenalties, globalPenaltyWeight);
        return;
    }

    std::vector<size_t> moved;
    std::vector<char> isMoved(count, 0);
    for (size_t index : move.partIndices) {
        if (index < count && !isMoved[index]) {
            isMoved[index] = 1;
            moved.push_back(index);
        }
    }
    if (moved.empty()) {
        return;
    }

    bool fullUsedBounds = false;
    for (size_t index : moved) {
        if (touchesUsedBoundary(partBounds_[index], usedBounds_, settings.collisionTolerance)) {
            fullUsedBounds = true;
        }
        const std::vector<size_t> oldPairs = pairsForPart(index);
        for (size_t other : oldPairs) {
            erasePairContribution(index, other);
        }
        const SheetScoreContribution oldSheet = sheetContributions_[index];
        invalidPartCount_ -= oldSheet.invalidPartCount;
        sheetPenalty_ -= oldSheet.sheetPenalty;
        removeBounds(index);
    }

    for (size_t index : moved) {
        transformedParts_[index] = transformPart(document.parts[index], state.poses[index], static_cast<int>(index));
        partBounds_[index] = transformedParts_[index].bounds;
        sheetContributions_[index] = evaluateSheet(document, settings, index, state.poses[index]);
        invalidPartCount_ += sheetContributions_[index].invalidPartCount;
        sheetPenalty_ += sheetContributions_[index].sheetPenalty;
        insertBounds(index, partBounds_[index].expanded(settings.partSpacing));
    }

    if (fullUsedBounds) {
        usedBounds_ = {};
        for (const AABB& bounds : partBounds_) {
            usedBounds_.include(bounds);
        }
    } else {
        for (size_t index : moved) {
            usedBounds_.include(partBounds_[index]);
        }
    }

    std::set<std::pair<size_t, size_t>> newPairs;
    for (size_t index : moved) {
        std::vector<size_t> neighbors = queryNeighbors(partBounds_[index].expanded(settings.partSpacing), index);
        for (size_t other : neighbors) {
            if (other >= count || other == index) {
                continue;
            }
            const size_t a = std::min(index, other);
            const size_t b = std::max(index, other);
            newPairs.insert({a, b});
        }
    }
    for (size_t i = 0; i < moved.size(); ++i) {
        for (size_t j = i + 1; j < moved.size(); ++j) {
            const size_t a = std::min(moved[i], moved[j]);
            const size_t b = std::max(moved[i], moved[j]);
            newPairs.insert({a, b});
        }
    }

    for (const auto& [a, b] : newPairs) {
        if (!partBounds_[a].expanded(settings.partSpacing).overlaps(partBounds_[b].expanded(settings.partSpacing))) {
            continue;
        }
        storePairContribution(a, b, evaluatePair(document, settings, transformedParts_[a], transformedParts_[b], a, b, *this));
    }

    collisionCount_ = std::max(0, collisionCount_);
    pairOverlapPenalty_ = std::max(0.0, pairOverlapPenalty_);
    spacingPenalty_ = std::max(0.0, spacingPenalty_);
    contactReward_ = std::max(0.0, contactReward_);
    invalidPartCount_ = std::max(0, invalidPartCount_);
    sheetPenalty_ = std::max(0.0, sheetPenalty_);
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
    double contactReward = cache.contactReward();
    int invalidPartCount = cache.invalidPartCount();
    double sheetPenalty = cache.sheetPenalty();

    for (size_t other : cache.pairsForPart(move.partIndex)) {
        const PairScoreContribution old = cache.pairContribution(move.partIndex, other);
        collisionCount -= old.collisionCount;
        overlapPenalty -= old.overlapPenalty;
        spacingPenalty -= old.spacingPenalty;
        contactReward -= old.contactReward;
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
        contactReward += contribution.contactReward;
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
    evaluation.totalScore = totalScore(document, collisionCount, std::max(0.0, overlapPenalty), invalidPartCount, evaluation.sheetPenalty, evaluation.spacingPenalty, used, cavityReward, std::max(0.0, contactReward), evaluation.usedWidth, evaluation.usedHeight);
    evaluation.valid = collisionCount == 0 && invalidPartCount == 0 && evaluation.spacingPenalty <= 0.0 && evaluation.sheetPenalty <= 0.0;
    return evaluation;
}

MultiDeltaEvaluation evaluateMultiMoveDelta(
    const Document& document,
    const EngineSettings& settings,
    const LayoutState& current,
    LayoutEvalCache& cache,
    const MultiDeltaMove& move) {
    MultiDeltaEvaluation evaluation;
    const size_t count = std::min(document.parts.size(), current.poses.size());
    if (count == 0 || cache.partBounds().size() < count || cache.transformedParts().size() < count ||
        move.partIndices.empty() || move.partIndices.size() != move.newPoses.size()) {
        return evaluation;
    }

    std::vector<size_t> moved;
    std::vector<char> isMoved(count, 0);
    std::unordered_map<size_t, Pose> newPoseByPart;
    for (size_t i = 0; i < move.partIndices.size(); ++i) {
        const size_t index = move.partIndices[i];
        if (index >= count) {
            return evaluation;
        }
        newPoseByPart[index] = move.newPoses[i];
        if (!isMoved[index]) {
            isMoved[index] = 1;
            moved.push_back(index);
        }
    }

    std::unordered_map<size_t, TransformedPart> movedTransformed;
    std::unordered_map<size_t, AABB> movedBounds;
    movedTransformed.reserve(moved.size());
    movedBounds.reserve(moved.size());
    for (size_t index : moved) {
        const Pose& pose = newPoseByPart[index];
        TransformedPart transformed = transformPart(document.parts[index], pose, static_cast<int>(index));
        movedBounds[index] = transformed.bounds;
        movedTransformed[index] = std::move(transformed);
    }

    std::vector<char> affected(count, 0);
    auto markAffected = [&](size_t index) {
        if (index < count) {
            affected[index] = 1;
        }
    };
    for (size_t index : moved) {
        markAffected(index);
        for (size_t other : cache.pairsForPart(index)) {
            markAffected(other);
        }
        for (size_t other : cache.queryNeighbors(cache.partBounds()[index].expanded(settings.partSpacing), index)) {
            markAffected(other);
        }
        for (size_t other : cache.queryNeighbors(movedBounds[index].expanded(settings.partSpacing), index)) {
            markAffected(other);
        }
    }

    std::set<std::pair<size_t, size_t>> pairSet;
    for (size_t index : moved) {
        for (size_t other = 0; other < count; ++other) {
            if (!affected[other] || other == index) {
                continue;
            }
            const size_t a = std::min(index, other);
            const size_t b = std::max(index, other);
            pairSet.insert({a, b});
        }
    }

    int collisionCount = cache.collisionCount();
    double overlapPenalty = cache.pairOverlapPenalty();
    double spacingPenalty = cache.spacingPenalty();
    double contactReward = cache.contactReward();
    int invalidPartCount = cache.invalidPartCount();
    double sheetPenalty = cache.sheetPenalty();

    for (const auto& [a, b] : pairSet) {
        const PairScoreContribution old = cache.pairContribution(a, b);
        collisionCount -= old.collisionCount;
        overlapPenalty -= old.overlapPenalty;
        spacingPenalty -= old.spacingPenalty;
        contactReward -= old.contactReward;
    }

    for (size_t index : moved) {
        const SheetScoreContribution oldSheet = cache.sheetContributions()[index];
        invalidPartCount -= oldSheet.invalidPartCount;
        sheetPenalty -= oldSheet.sheetPenalty;
    }

    auto transformedFor = [&](size_t index) -> const TransformedPart& {
        auto it = movedTransformed.find(index);
        return it == movedTransformed.end() ? cache.transformedParts()[index] : it->second;
    };
    auto poseFor = [&](size_t index) -> const Pose& {
        auto it = newPoseByPart.find(index);
        return it == newPoseByPart.end() ? current.poses[index] : it->second;
    };

    for (const auto& [a, b] : pairSet) {
        const TransformedPart& transformedA = transformedFor(a);
        const TransformedPart& transformedB = transformedFor(b);
        if (!transformedA.bounds.expanded(settings.partSpacing).overlaps(transformedB.bounds.expanded(settings.partSpacing))) {
            continue;
        }
        const PairScoreContribution contribution = evaluatePair(document, settings, transformedA, transformedB, a, b, cache);
        collisionCount += contribution.collisionCount;
        overlapPenalty += contribution.overlapPenalty;
        spacingPenalty += contribution.spacingPenalty;
        contactReward += contribution.contactReward;
    }

    for (size_t index : moved) {
        const SheetScoreContribution newSheet = evaluateSheet(document, settings, index, poseFor(index));
        invalidPartCount += newSheet.invalidPartCount;
        sheetPenalty += newSheet.sheetPenalty;
    }

    bool fullUsedBounds = false;
    for (size_t index : moved) {
        if (touchesUsedBoundary(cache.partBounds()[index], cache.usedBounds(), settings.collisionTolerance)) {
            fullUsedBounds = true;
            break;
        }
    }

    AABB used = cache.usedBounds();
    if (fullUsedBounds) {
        used = {};
        for (size_t i = 0; i < count; ++i) {
            auto movedIt = movedBounds.find(i);
            used.include(movedIt == movedBounds.end() ? cache.partBounds()[i] : movedIt->second);
        }
    } else {
        for (size_t index : moved) {
            used.include(movedBounds[index]);
        }
    }

    std::vector<Pose> candidatePoses = current.poses;
    for (size_t index : moved) {
        candidatePoses[index] = newPoseByPart[index];
    }

    const double cavityReward = cavityPlacementReward(document, candidatePoses);
    evaluation.collisionCount = std::max(0, collisionCount);
    evaluation.invalidPartCount = std::max(0, invalidPartCount);
    evaluation.spacingPenalty = std::max(0.0, spacingPenalty);
    evaluation.sheetPenalty = std::max(0.0, sheetPenalty);
    evaluation.affectedPartCount = 0;
    for (char value : affected) {
        evaluation.affectedPartCount += value ? 1u : 0u;
    }
    evaluation.affectedPairCount = pairSet.size();
    evaluation.totalScore = totalScore(
        document,
        evaluation.collisionCount,
        std::max(0.0, overlapPenalty),
        evaluation.invalidPartCount,
        evaluation.sheetPenalty,
        evaluation.spacingPenalty,
        used,
        cavityReward,
        std::max(0.0, contactReward),
        evaluation.usedWidth,
        evaluation.usedHeight);
    evaluation.valid = evaluation.collisionCount == 0 &&
        evaluation.invalidPartCount == 0 &&
        evaluation.spacingPenalty <= 0.0 &&
        evaluation.sheetPenalty <= 0.0;
    return evaluation;
}

} // namespace nest
