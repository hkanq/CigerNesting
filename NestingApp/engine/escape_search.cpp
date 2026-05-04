#include "engine/escape_search.h"

#include "engine/free_space_analyzer.h"
#include "engine/layout_score.h"
#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace nest {
namespace {

double footprint(const Part& part) {
    return std::max(1.0, part.localBounds.area());
}

AABB orientedLocalBounds(const Part& part, const Pose& orientation) {
    Pose local;
    local.angleRadians = orientation.angleRadians;
    local.mirrored = orientation.mirrored;
    const Transform transform = local.toTransform();
    AABB bounds;
    for (const Ring& ring : part.rings) {
        for (const Vec2& point : ring.points) {
            bounds.include(transform.apply(point));
        }
    }
    return bounds;
}

Pose centeredPose(const Part& part, const Pose& orientation, Vec2 anchor) {
    const AABB bounds = orientedLocalBounds(part, orientation);
    Pose out = orientation;
    out.x = anchor.x - bounds.center().x;
    out.y = anchor.y - bounds.center().y;
    return out;
}

std::vector<size_t> escapeTargets(const Document& document, const LayoutState& state, const EngineSettings& settings, unsigned int seed) {
    std::vector<std::pair<double, size_t>> ranked;
    ranked.reserve(document.parts.size());
    const Vec2 sheetCenter{
        document.sheet.origin.x + document.sheet.width * 0.5,
        document.sheet.origin.y + document.sheet.height * 0.5
    };
    for (size_t i = 0; i < document.parts.size() && i < state.poses.size(); ++i) {
        const AABB bounds = transformedBounds(document.parts[i], state.poses[i]);
        const double centerBias = -distance(bounds.center(), sheetCenter) * 0.02;
        const double jitter = static_cast<double>((seed + static_cast<unsigned int>(i * 977u)) % 17u) * 0.001;
        ranked.push_back({footprint(document.parts[i]) + centerBias + jitter, i});
    }
    std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) {
        return a.first < b.first;
    });
    size_t limit = 2;
    if (settings.performanceProfile == PerformanceProfile::Balanced) {
        limit = 3;
    } else if (settings.performanceProfile == PerformanceProfile::Maximum) {
        limit = 5;
    }
    limit = std::min(limit, ranked.size());
    std::vector<size_t> out;
    out.reserve(limit);
    for (size_t i = 0; i < limit; ++i) {
        out.push_back(ranked[i].second);
    }
    return out;
}

std::vector<Vec2> escapeAnchors(const Document& document, const EngineSettings& settings, const LayoutState& state) {
    FreeSpaceAnalyzer analyzer;
    const std::vector<FreeSpaceCandidate> candidates = analyzer.analyze(document, settings, state);
    std::vector<Vec2> anchors;
    anchors.reserve(candidates.size() + 12);
    const double left = document.sheet.origin.x + settings.margin;
    const double right = document.sheet.origin.x + document.sheet.width - settings.margin;
    const double bottom = document.sheet.origin.y + settings.margin;
    const double top = document.sheet.origin.y + document.sheet.height - settings.margin;
    const Vec2 center{(left + right) * 0.5, (bottom + top) * 0.5};
    const Vec2 fixed[] = {
        {left, bottom}, {right, bottom}, {right, top}, {left, top},
        {center.x, bottom}, {right, center.y}, {center.x, top}, {left, center.y},
        {(left + center.x) * 0.5, (bottom + center.y) * 0.5},
        {(right + center.x) * 0.5, (bottom + center.y) * 0.5},
        {(right + center.x) * 0.5, (top + center.y) * 0.5},
        {(left + center.x) * 0.5, (top + center.y) * 0.5}
    };
    for (const Vec2& anchor : fixed) {
        anchors.push_back(anchor);
    }
    for (const FreeSpaceCandidate& candidate : candidates) {
        if (candidate.kind == FreeSpaceCandidateKind::SheetCorner ||
            candidate.kind == FreeSpaceCandidateKind::SheetBoundary ||
            candidate.kind == FreeSpaceCandidateKind::UsedBoundsGap ||
            candidate.kind == FreeSpaceCandidateKind::PartOuter) {
            anchors.push_back(candidate.anchor);
        }
    }
    return anchors;
}

size_t attemptBudget(const EngineSettings& settings) {
    if (settings.performanceProfile == PerformanceProfile::Fast) {
        return 24;
    }
    if (settings.performanceProfile == PerformanceProfile::Maximum) {
        return 160;
    }
    return 72;
}

} // namespace

LayoutState EscapeSearch::escape(
    const Document& document,
    const EngineSettings& settings,
    LayoutState state,
    const PenaltySystem& penalties,
    const std::atomic_bool& stopRequested,
    unsigned int seed,
    SolverStats* stats) const {
    LayoutScore scorer;
    state = scorer.evaluate(document, settings, state.poses, &penalties);
    if (!state.valid() || document.parts.empty()) {
        return state;
    }

    const std::vector<size_t> targets = escapeTargets(document, state, settings, seed);
    std::vector<Vec2> anchors = escapeAnchors(document, settings, state);
    if (targets.empty() || anchors.empty()) {
        return state;
    }

    std::mt19937 rng(seed ^ 0xa5a5a5a5u);
    std::shuffle(anchors.begin(), anchors.end(), rng);
    const size_t budget = attemptBudget(settings);
    size_t attempts = 0;
    for (size_t target : targets) {
        if (target >= document.parts.size() || target >= state.poses.size()) {
            continue;
        }
        const Vec2 oldCenter = transformedBounds(document.parts[target], state.poses[target]).center();
        std::sort(anchors.begin(), anchors.end(), [&](const Vec2& a, const Vec2& b) {
            return distance(a, oldCenter) > distance(b, oldCenter);
        });
        for (const Vec2& anchor : anchors) {
            if (stopRequested.load() || attempts >= budget) {
                return state;
            }
            ++attempts;
            if (stats) {
                ++stats->escapeAttempts;
                ++stats->evaluatedCandidates;
            }
            std::vector<Pose> poses = state.poses;
            poses[target] = centeredPose(document.parts[target], state.poses[target], anchor);
            LayoutState trial = scorer.evaluate(document, settings, poses, &penalties);
            if (!trial.valid()) {
                continue;
            }
            if (stats) {
                ++stats->escapeAccepted;
                ++stats->acceptedMoves;
                if (trial.totalScore > state.totalScore + 1e-9) {
                    ++stats->acceptedWorseMoves;
                } else if (trial.totalScore + 1e-9 < state.totalScore) {
                    ++stats->bestUpdates;
                }
            }
            return trial;
        }
    }
    return state;
}

} // namespace nest
