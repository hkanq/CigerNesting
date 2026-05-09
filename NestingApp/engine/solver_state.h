#pragma once

#include "core/polygon.h"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace nest {

enum class SolverPhase {
    Idle,
    PrepareGeometry,
    InitialPlacement,
    Exploration,
    CollisionResolution,
    Compression,
    ContactPacking,
    GapFilling,
    Rearrangement,
    Escape,
    UltraRefinement,
    FinalValidation,
    Done,
    NoValidLayout,
    Failed,
    Stopped
};

enum class SolverStrategy {
    Idle,
    AdaptiveSearch,
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
    Frontier,
    Done
};

inline constexpr size_t kNoPartIndex = static_cast<size_t>(-1);

struct ActiveMoveSummary {
    size_t contact = 0;
    size_t compression = 0;
    size_t gap = 0;
    size_t hole = 0;
    size_t concavity = 0;
    size_t smallPart = 0;
    size_t swap = 0;
    size_t chain = 0;
    size_t cluster = 0;
    size_t region = 0;
    size_t rotation = 0;
    size_t mirror = 0;
    size_t escape = 0;
    size_t frontier = 0;
};

struct SolverStats {
    size_t evaluatedCandidates = 0;
    size_t acceptedMoves = 0;
    size_t rejectedCollision = 0;
    size_t rejectedSpacing = 0;
    size_t rejectedSheet = 0;
    size_t attemptsStarted = 0;
    size_t attemptsCompleted = 0;
    size_t bestUpdates = 0;
    size_t gapAccepted = 0;
    double elapsedMs = 0.0;
    size_t workerCount = 0;
    double candidatesPerSecond = 0.0;
    size_t cacheHits = 0;
    size_t cacheMisses = 0;
    size_t swapAttempts = 0;
    size_t swapAccepted = 0;
    size_t chainAttempts = 0;
    size_t chainAccepted = 0;
    size_t clusterAttempts = 0;
    size_t clusterAccepted = 0;
    size_t acceptedWorseMoves = 0;
    size_t rejectedWorseMoves = 0;
    size_t tabuRejected = 0;
    size_t escapeAttempts = 0;
    size_t escapeAccepted = 0;
    size_t ultraAccepted = 0;
    size_t compactionAccepted = 0;
    size_t frontierCandidates = 0;
    size_t smallFillerAccepted = 0;
    size_t regionRepackAccepted = 0;
    size_t rowBaselineUsed = 0;
    size_t contourSeedUsed = 0;
    size_t rowFallbackUsed = 0;
    double emptySpaceArea = 0.0;
    double largestEmptyRegionArea = 0.0;
    size_t fillableGapCount = 0;
    size_t contactCount = 0;
    double averageClearance = 0.0;
    size_t slideToContactAccepted = 0;
    size_t aggressiveGapAccepted = 0;
    size_t localRegionRepackAttempts = 0;
    size_t localRegionRepackAccepted = 0;
    size_t localRegionRepackSubsets = 0;
    size_t localRegionRepackCandidatesGenerated = 0;
    size_t localRegionRepackValidCandidates = 0;
    size_t localRegionRepackNoCandidate = 0;
    size_t localRegionRepackCollisionReject = 0;
    size_t localRegionRepackClearanceReject = 0;
    size_t localRegionRepackSheetReject = 0;
    size_t localRegionRepackScoreReject = 0;
    size_t localRegionRepackBeamPruned = 0;
    size_t localRegionRepackFullValidationReject = 0;
    size_t localRegionRepackMaxCandidatesForPart = 0;
    size_t analyticCandidatesGenerated = 0;
    size_t analyticCandidatesValid = 0;
    size_t analyticFallbackCandidatesGenerated = 0;
    size_t analyticFallbackCandidatesValid = 0;
    size_t analyticCandidatesAccepted = 0;
    size_t nfpCandidatesGenerated = 0;
    size_t nfpCandidatesValid = 0;
    size_t nfpCandidatesAccepted = 0;
    size_t nfpLoopsGenerated = 0;
    size_t nfpLoopCandidatesGenerated = 0;
    size_t ifpCandidatesGenerated = 0;
    size_t ifpCandidatesValid = 0;
    size_t ifpCandidatesAccepted = 0;
    size_t ifpLoopsGenerated = 0;
    size_t nfpCacheHits = 0;
    size_t nfpCacheMisses = 0;
    size_t contactCandidatesRejectedCollision = 0;
    size_t contactCandidatesRejectedClearance = 0;
    size_t contactCandidatesRejectedSheet = 0;
    size_t contactCandidatesRejectedScore = 0;
    size_t contourContactAccepted = 0;
    size_t activeMoveAcceptedTotal = 0;
    size_t acceptedBetter = 0;
    size_t acceptedTemporary = 0;
    size_t rejectedByScore = 0;
    size_t rejectedByAcceptance = 0;
    double acceptanceRate = 0.0;
    size_t destroyAttempts = 0;
    size_t destroyAccepted = 0;
    size_t destroyTemporaryAccepted = 0;
    size_t destroyTemporaryAcceptedWithObjectiveGain = 0;
    size_t destroyTemporaryRejectedNoObjectiveGain = 0;
    size_t destroyBestUpdates = 0;
    size_t destroyRejectedInvalid = 0;
    size_t destroyRejectedWorseAllMetrics = 0;
    size_t destroyAcceptedReducedUsedBounds = 0;
    size_t destroyAcceptedReducedLargestEmptyRegion = 0;
    size_t destroyAcceptedReducedTotalEmptyArea = 0;
    size_t destroyAcceptedIncreasedContactWithGapReduction = 0;
    size_t rebuildCompactionAttempts = 0;
    size_t rebuildCompactionClusters = 0;
    size_t rebuildCompactionAccepted = 0;
    size_t coordinatedClusterRebuildAttempts = 0;
    size_t coordinatedClusterRebuildAccepted = 0;
    size_t coordinatedClusterMotionAccepted = 0;
    size_t denseSmallPartCompactionAttempts = 0;
    size_t denseSmallPartCompactionAccepted = 0;
    size_t clusterBeamStatesGenerated = 0;
    size_t clusterBeamStatesKept = 0;
    size_t clusterBeamLeaves = 0;
    size_t clusterBeamAccepted = 0;
    size_t clusterBeamRestoreFallbackCount = 0;
    size_t clusterBeamDepthTotal = 0;
    double clusterBeamAverageDepth = 0.0;
    double clusterBeamRestoreFallbackRatio = 0.0;
    size_t denseClusterBeamAccepted = 0;
    size_t coordinatedClusterSizeTotal = 0;
    double averageCoordinatedClusterSize = 0.0;
    size_t destroySubsetTotal = 0;
    double averageSubsetSize = 0.0;
    size_t placementDepthTotal = 0;
    double averagePlacementDepth = 0.0;
    size_t activeContactDepthTotal = 0;
    double averageActiveContactDepth = 0.0;
    size_t expansionLimitTotal = 0;
    double averageExpansionLimit = 0.0;
    size_t partialEvalLimitTotal = 0;
    double averagePartialEvalLimit = 0.0;
    size_t rebuildPreviewEvents = 0;
    size_t beamNodesExpanded = 0;
    size_t beamValidLeaves = 0;
    double rebuildBeforeUtilization = 0.0;
    double rebuildAfterUtilization = 0.0;
    double rebuildBeforeUsedArea = 0.0;
    double rebuildAfterUsedArea = 0.0;
    double rebuildBeforeLargestEmptyRegion = 0.0;
    double rebuildAfterLargestEmptyRegion = 0.0;
    double rebuildBeforeTotalEmptyArea = 0.0;
    double rebuildAfterTotalEmptyArea = 0.0;
    double rebuildBeforeContactCount = 0.0;
    double rebuildAfterContactCount = 0.0;
    double rebuildBeforeAverageClearance = 0.0;
    double rebuildAfterAverageClearance = 0.0;
    double rebuildBeforeTowerScore = 0.0;
    double rebuildAfterTowerScore = 0.0;
    double bestRebuildUsedAreaReduction = 0.0;
    double bestRebuildUsedWidthReduction = 0.0;
    double bestRebuildUsedHeightReduction = 0.0;
    double bestRebuildUtilizationGain = 0.0;
    double bestRebuildLargestEmptyRegionReduction = 0.0;
    double bestRebuildTotalEmptyAreaReduction = 0.0;
    double bestRebuildContactGain = 0.0;
    double towerScore = 0.0;
    double layoutSpreadScore = 0.0;
    double unusedSheetRegionScore = 0.0;
    size_t lowContactPartCount = 0;
    size_t qualityFailReason = 0;
    ActiveMoveSummary activeMoveSummary;
    ActiveMoveSummary acceptedMoveSummary;
};

struct SolverSnapshot {
    std::vector<Pose> currentPoses;
    std::vector<Pose> bestPoses;
    SolverPhase phase = SolverPhase::Idle;
    double progress = 0.0;
    size_t collisionCount = 0;
    double overlapScore = 0.0;
    double utilization = 0.0;
    double elapsedSeconds = 0.0;
    size_t validationFailureCount = 0;
    size_t invalidPartCount = 0;
    SolverStats stats;
    SolverStrategy currentStrategy = SolverStrategy::Idle;
    ActiveMoveSummary activeMoves;
    uint64_t versionId = 0;
    bool layoutChanged = false;
    size_t lastMovedPart = kNoPartIndex;
    SolverStrategy lastMoveStrategy = SolverStrategy::Idle;
    bool bestUpdated = false;
    std::vector<size_t> changedParts;
    size_t rebuildAttempt = 0;
    size_t beamDepth = 0;
    size_t subsetSize = 0;
    bool previewTemporary = false;
    double bestValidUtilization = 0.0;
    double currentTrajectoryUtilization = 0.0;
    size_t bestUpdateCount = 0;
    size_t destroyBestUpdateCount = 0;
    bool running = false;
};

struct SolverResult {
    std::vector<Pose> bestPoses;
    size_t collisionCount = 0;
    double overlapScore = 0.0;
    double utilization = 0.0;
    size_t validationFailureCount = 0;
    size_t invalidPartCount = 0;
    SolverStats stats;
    SolverStrategy currentStrategy = SolverStrategy::Idle;
    uint64_t versionId = 0;
    bool valid = false;
};

} // namespace nest
