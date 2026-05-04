#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_state.h"
#include "engine/penalty_system.h"
#include "engine/solver_state.h"
#include <atomic>
#include <chrono>
#include <cstddef>
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
    size_t neighborCount = 0;
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

using AdaptiveProgressCallback = std::function<void(SolverStrategy, const LayoutState&, const LayoutState&, const SolverStats&)>;

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
