#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include <cstddef>
#include <vector>

namespace nest {

enum class AnalyticContactKind {
    MovingVertexToFixedVertex,
    MovingVertexToFixedEdge,
    MovingEdgeToFixedVertex,
    ConvexToConcaveVertex,
    HoleBoundary,
    SheetBoundary,
    EdgeParallel,
    NotchCavity,
    RegionAnchor,
    NfpPartPart,
    NfpHoleBoundary,
    InnerFitBoundary
};

struct AnalyticContactCandidate {
    Pose pose;
    AnalyticContactKind kind = AnalyticContactKind::MovingVertexToFixedVertex;
    size_t sourcePart = static_cast<size_t>(-1);
    int sourceRing = -1;
    double priority = 0.0;
};

struct AnalyticContactStats {
    size_t generated = 0;
    size_t valid = 0;
    size_t rejectedCollision = 0;
    size_t rejectedClearance = 0;
    size_t rejectedSheet = 0;
    size_t cacheHits = 0;
    size_t cacheMisses = 0;
    size_t nfpLoopsGenerated = 0;
    size_t nfpLoopCandidatesGenerated = 0;
    size_t ifpLoopsGenerated = 0;
};

struct AnalyticContactRequest {
    size_t movingPart = 0;
    std::vector<size_t> fixedParts;
    std::vector<Vec2> regionAnchors;
    std::vector<double> angles;
    std::vector<bool> mirrors;
    size_t ownerLimit = 32;
    size_t perOwnerPointLimit = 8;
    size_t candidateLimit = 128;
    bool includeSheetBoundary = true;
    bool includeHoleContacts = true;
    bool includeRegionAnchors = true;
};

class AnalyticContactCandidateGenerator {
public:
    std::vector<AnalyticContactCandidate> generate(
        const Document& document,
        const EngineSettings& settings,
        const std::vector<Pose>& poses,
        const AnalyticContactRequest& request,
        AnalyticContactStats* stats = nullptr) const;
};

} // namespace nest

