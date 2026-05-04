#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_state.h"
#include "engine/penalty_system.h"
#include "engine/solver_state.h"
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

namespace nest {

using PartId = size_t;

enum class OperatorKind {
    ContactPacking,
    Compression,
    GapFilling,
    HoleFilling,
    ConcavityFilling,
    SmallPartFiller,
    Swap,
    EjectionChain,
    ClusterRepack,
    RegionRepack,
    RotationRefinement,
    Mirror,
    Escape,
    Frontier
};

struct CandidateMove {
    PartId part = 0;
    Pose newPose;
    double estimatedDeltaScore = 0.0;
    OperatorKind source = OperatorKind::Compression;
    std::vector<PartId> parts;
    std::vector<Pose> newPoses;

    bool isMultiPart() const {
        return parts.size() > 1 && parts.size() == newPoses.size();
    }
};

struct PartNeed {
    double needsCompression = 0.0;
    double needsRotation = 0.0;
    double needsMirror = 0.0;
    double needsHoleFilling = 0.0;
    double needsConcavityFitting = 0.0;
    double needsGapMove = 0.0;
    double needsSwap = 0.0;
    double needsEscape = 0.0;
    double blocksOthers = 0.0;
    double wastedSpaceAround = 0.0;
    double boundaryContribution = 0.0;
    double mobilityScore = 0.0;
};

struct PartState {
    PartId part = 0;
    bool nearBoundary = false;
    bool hasCollision = false;
    bool highClearanceGap = false;
    bool inDenseRegion = false;
    bool candidateForHole = false;
    bool candidateForConcavity = false;
    bool rotationSensitive = false;
    bool mirrorSensitive = false;
    double priority = 0.0;
    double sizeRank = 0.0;
    double potentialScore = 0.0;
    size_t neighborCount = 0;
    PartNeed need;
};

struct MoveTask {
    PartId partIndex = 0;
    OperatorKind operatorType = OperatorKind::Compression;
    double priority = 0.0;
    double estimatedGain = 0.0;
};

struct OperatorStats {
    int attempts = 0;
    int accepted = 0;
    double totalImprovement = 0.0;
    double lastImprovementTime = 0.0;

    double schedulerScore() const {
        const double attemptScale = attempts > 0 ? static_cast<double>(attempts) : 1.0;
        const double success = static_cast<double>(accepted + 1) / (attemptScale + 2.0);
        return success * (1.0 + totalImprovement);
    }
};

struct ConvergenceState {
    int noImprovementSteps = 0;
    double lastBestScore = 0.0;
    int repeatedLayouts = 0;
    int lowAcceptanceSteps = 0;
    size_t lastAcceptedMoves = 0;
    size_t highPotentialParts = 0;
    size_t promisingTasks = 0;
};

class IOperator {
public:
    virtual ~IOperator() = default;
    virtual OperatorKind kind() const = 0;
    virtual void generateCandidates(
        const LayoutState& state,
        const PartState& part,
        std::vector<CandidateMove>& out) = 0;
};

struct AdaptiveProgressEvent {
    SolverStrategy strategy = SolverStrategy::AdaptiveSearch;
    LayoutState current;
    LayoutState best;
    SolverStats stats;
    ActiveMoveSummary activeMoves;
    uint64_t versionId = 0;
    bool layoutChanged = false;
    size_t lastMovedPart = kNoPartIndex;
    SolverStrategy lastMoveStrategy = SolverStrategy::Idle;
    bool bestUpdated = false;
};

using AdaptiveProgressCallback = std::function<void(const AdaptiveProgressEvent&)>;

SolverStrategy strategyForOperator(OperatorKind kind);

class AdaptiveUnifiedOptimizer {
public:
    LayoutState optimize(
        const Document& document,
        const EngineSettings& settings,
        LayoutState initialValid,
        PenaltySystem& globalPenalties,
        const std::atomic_bool& stopRequested,
        SolverStats& stats,
        AdaptiveProgressCallback callback) const;

private:
    using Clock = std::chrono::steady_clock;

    mutable std::unordered_map<OperatorKind, OperatorStats> operatorStats_;
};

} // namespace nest
