#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_state.h"
#include "engine/penalty_system.h"
#include "geometry/transformed_shape.h"
#include <cstddef>
#include <unordered_map>
#include <vector>

namespace nest {

struct DeltaMove {
    size_t partIndex = 0;
    Pose oldPose;
    Pose newPose;
};

struct DeltaEvaluation {
    bool valid = false;
    double totalScore = 0.0;
    int collisionCount = 0;
    int invalidPartCount = 0;
    double spacingPenalty = 0.0;
    double sheetPenalty = 0.0;
    double usedWidth = 0.0;
    double usedHeight = 0.0;
};

struct PairScoreContribution {
    int collisionCount = 0;
    double overlapPenalty = 0.0;
    double spacingPenalty = 0.0;
};

struct SheetScoreContribution {
    int invalidPartCount = 0;
    double sheetPenalty = 0.0;
};

class LayoutEvalCache {
public:
    void rebuild(
        const Document& document,
        const EngineSettings& settings,
        const LayoutState& state,
        const PenaltySystem* attemptPenalties = nullptr,
        const PenaltySystem* globalPenalties = nullptr,
        double globalPenaltyWeight = 0.10);

    void updateAfterAcceptedMove(
        const Document& document,
        const EngineSettings& settings,
        const LayoutState& state,
        const DeltaMove& move,
        const PenaltySystem* attemptPenalties = nullptr,
        const PenaltySystem* globalPenalties = nullptr,
        double globalPenaltyWeight = 0.10);

    std::vector<size_t> queryNeighbors(const AABB& bounds, size_t excludedPart) const;
    PairScoreContribution pairContribution(size_t a, size_t b) const;
    const std::vector<size_t>& pairsForPart(size_t index) const;

    const std::vector<TransformedPart>& transformedParts() const { return transformedParts_; }
    const std::vector<AABB>& partBounds() const { return partBounds_; }
    const std::vector<SheetScoreContribution>& sheetContributions() const { return sheetContributions_; }
    const AABB& usedBounds() const { return usedBounds_; }
    double pairOverlapPenalty() const { return pairOverlapPenalty_; }
    double spacingPenalty() const { return spacingPenalty_; }
    double sheetPenalty() const { return sheetPenalty_; }
    int collisionCount() const { return collisionCount_; }
    int invalidPartCount() const { return invalidPartCount_; }
    const PenaltySystem* attemptPenalties() const { return attemptPenalties_; }
    const PenaltySystem* globalPenalties() const { return globalPenalties_; }
    double globalPenaltyWeight() const { return globalPenaltyWeight_; }
    double cellSize() const { return cellSize_; }

private:
    struct PairKey {
        size_t a = 0;
        size_t b = 0;

        bool operator==(const PairKey& other) const {
            return a == other.a && b == other.b;
        }
    };

    struct PairKeyHash {
        size_t operator()(const PairKey& key) const {
            return key.a ^ (key.b + 0x9e3779b97f4a7c15ull + (key.a << 6) + (key.a >> 2));
        }
    };

    struct CellKey {
        int x = 0;
        int y = 0;

        bool operator==(const CellKey& other) const {
            return x == other.x && y == other.y;
        }
    };

    struct CellKeyHash {
        size_t operator()(const CellKey& key) const {
            return static_cast<size_t>(static_cast<unsigned int>(key.x) * 73856093u) ^
                static_cast<size_t>(static_cast<unsigned int>(key.y) * 19349663u);
        }
    };

    static PairKey makePairKey(size_t a, size_t b);
    void rebuildSpatialIndex(double spacing);
    void insertBounds(size_t index, const AABB& bounds);

    std::vector<TransformedPart> transformedParts_;
    std::vector<AABB> partBounds_;
    std::vector<SheetScoreContribution> sheetContributions_;
    std::vector<std::vector<size_t>> pairsByPart_;
    std::unordered_map<PairKey, PairScoreContribution, PairKeyHash> pairContributions_;
    std::unordered_map<CellKey, std::vector<size_t>, CellKeyHash> grid_;
    AABB usedBounds_;
    double pairOverlapPenalty_ = 0.0;
    double spacingPenalty_ = 0.0;
    double sheetPenalty_ = 0.0;
    int collisionCount_ = 0;
    int invalidPartCount_ = 0;
    double cellSize_ = 64.0;
    const PenaltySystem* attemptPenalties_ = nullptr;
    const PenaltySystem* globalPenalties_ = nullptr;
    double globalPenaltyWeight_ = 0.10;
};

DeltaEvaluation evaluateMoveDelta(
    const Document& document,
    const EngineSettings& settings,
    const LayoutState& current,
    const LayoutEvalCache& cache,
    const DeltaMove& move);

} // namespace nest
