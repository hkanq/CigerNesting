#include "engine/region_repack.h"

#include "engine/frontier_analyzer.h"
#include "engine/layout_eval_cache.h"
#include "engine/layout_score.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace nest {
namespace {

struct CellInfo {
    AABB bounds;
    double occupied = 0.0;
    double ratio = 0.0;
};

double intersectionArea(const AABB& a, const AABB& b) {
    if (!a.isValid() || !b.isValid()) {
        return 0.0;
    }
    const double x = std::max(0.0, std::min(a.max.x, b.max.x) - std::max(a.min.x, b.min.x));
    const double y = std::max(0.0, std::min(a.max.y, b.max.y) - std::max(a.min.y, b.min.y));
    return x * y;
}

Pose poseWithMinAt(const Part& part, const Pose& base, Vec2 anchor) {
    Pose pose = base;
    const AABB local = transformedBounds(part, Pose{0.0, 0.0, base.angleRadians, base.mirrored});
    pose.x = anchor.x - local.min.x;
    pose.y = anchor.y - local.min.y;
    return pose;
}

bool samePose(const Pose& a, const Pose& b) {
    return std::abs(a.x - b.x) < 1e-9 &&
        std::abs(a.y - b.y) < 1e-9 &&
        std::abs(a.angleRadians - b.angleRadians) < 1e-9 &&
        a.mirrored == b.mirrored;
}

std::vector<CellInfo> sparseCells(const Document& document, const LayoutState& state, const EngineSettings& settings, AABB& used) {
    const size_t count = std::min(document.parts.size(), state.poses.size());
    std::vector<AABB> boxes;
    boxes.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        boxes.push_back(transformedBounds(document.parts[i], state.poses[i]));
        used.include(boxes.back());
    }
    if (!used.isValid() || used.width() <= 1e-9 || used.height() <= 1e-9) {
        return {};
    }

    const int columns = settings.performanceProfile == PerformanceProfile::Maximum ? 8 : 6;
    const int rows = settings.performanceProfile == PerformanceProfile::Maximum ? 6 : 4;
    std::vector<CellInfo> cells;
    cells.reserve(static_cast<size_t>(columns * rows));
    const double cellW = used.width() / static_cast<double>(columns);
    const double cellH = used.height() / static_cast<double>(rows);
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < columns; ++x) {
            CellInfo cell;
            cell.bounds = AABB::fromMinMax(
                {used.min.x + static_cast<double>(x) * cellW, used.min.y + static_cast<double>(y) * cellH},
                {used.min.x + static_cast<double>(x + 1) * cellW, used.min.y + static_cast<double>(y + 1) * cellH});
            for (const AABB& box : boxes) {
                cell.occupied += intersectionArea(cell.bounds, box);
            }
            cell.ratio = cell.occupied / std::max(1.0, cell.bounds.area());
            if (cell.ratio < 0.35) {
                cells.push_back(cell);
            }
        }
    }
    std::sort(cells.begin(), cells.end(), [](const CellInfo& a, const CellInfo& b) {
        return a.ratio < b.ratio;
    });
    if (cells.size() > 12) {
        cells.resize(12);
    }
    return cells;
}

std::vector<size_t> affectedParts(const Document& document, const LayoutState& state, const AABB& used, const std::vector<CellInfo>& cells, const EngineSettings& settings) {
    struct Candidate {
        size_t index = 0;
        double rank = 0.0;
    };
    std::vector<Candidate> ranked;
    const size_t count = std::min(document.parts.size(), state.poses.size());
    const double eps = std::max(1.0, settings.partSpacing + settings.collisionTolerance);
    for (size_t i = 0; i < count; ++i) {
        const AABB box = transformedBounds(document.parts[i], state.poses[i]);
        const bool boundary = used.isValid() && (std::abs(box.max.x - used.max.x) <= eps || std::abs(box.max.y - used.max.y) <= eps);
        double proximity = boundary ? 0.0 : 100000.0;
        for (const CellInfo& cell : cells) {
            proximity = std::min(proximity, distance(box.center(), cell.bounds.center()));
        }
        const double areaRank = std::max(1.0, document.parts[i].localBounds.area()) * 0.001;
        ranked.push_back({i, proximity + areaRank - (boundary ? 5000.0 : 0.0)});
    }
    std::sort(ranked.begin(), ranked.end(), [](const Candidate& a, const Candidate& b) {
        return a.rank < b.rank;
    });
    const size_t limit = settings.performanceProfile == PerformanceProfile::Maximum ? 32u :
        settings.performanceProfile == PerformanceProfile::Balanced ? 18u : 8u;
    std::vector<size_t> out;
    for (size_t i = 0; i < ranked.size() && i < limit; ++i) {
        out.push_back(ranked[i].index);
    }
    return out;
}

void classifyRejected(const DeltaEvaluation& evaluation, SolverStats* stats) {
    if (!stats) {
        return;
    }
    if (evaluation.collisionCount > 0) {
        ++stats->rejectedCollision;
    } else if (evaluation.spacingPenalty > 0.0) {
        ++stats->rejectedSpacing;
    } else if (evaluation.invalidPartCount > 0 || evaluation.sheetPenalty > 0.0) {
        ++stats->rejectedSheet;
    }
}

} // namespace

LayoutState RegionRepack::improve(
    const Document& document,
    const EngineSettings& settings,
    LayoutState state,
    const PenaltySystem& penalties,
    const std::atomic_bool& stopRequested,
    SolverStats* stats) const {
    if (document.parts.empty() || state.poses.size() < document.parts.size()) {
        return state;
    }

    LayoutScore scorer;
    state = scorer.evaluate(document, settings, state.poses, &penalties);
    if (!state.valid()) {
        return state;
    }

    AABB used;
    const std::vector<CellInfo> cells = sparseCells(document, state, settings, used);
    if (cells.empty()) {
        return state;
    }

    FrontierAnalyzer analyzer;
    std::vector<FrontierCandidate> frontiers = analyzer.analyze(document, settings, state);
    if (stats) {
        stats->frontierCandidates += frontiers.size();
    }
    for (const CellInfo& cell : cells) {
        frontiers.push_back({cell.bounds.min, FrontierCandidateKind::EmptyCell, static_cast<size_t>(-1), 45.0 * (1.0 - cell.ratio), cell.bounds});
        frontiers.push_back({cell.bounds.center(), FrontierCandidateKind::EmptyCell, static_cast<size_t>(-1), 42.0 * (1.0 - cell.ratio), cell.bounds});
    }
    std::stable_sort(frontiers.begin(), frontiers.end(), [](const FrontierCandidate& a, const FrontierCandidate& b) {
        return a.priority > b.priority;
    });

    const std::vector<size_t> parts = affectedParts(document, state, used, cells, settings);
    size_t maxEvaluations = settings.performanceProfile == PerformanceProfile::Maximum ? 24000u :
        settings.performanceProfile == PerformanceProfile::Balanced ? 10000u : 2500u;
    if (document.parts.size() > 300) {
        maxEvaluations = std::min<size_t>(maxEvaluations, settings.performanceProfile == PerformanceProfile::Maximum ? 9000u : 3500u);
    } else if (document.parts.size() > 100) {
        maxEvaluations = std::min<size_t>(maxEvaluations, settings.performanceProfile == PerformanceProfile::Maximum ? 12000u : 5500u);
    }
    size_t evaluations = 0;
    LayoutEvalCache cache;
    cache.rebuild(document, settings, state, &penalties);

    for (size_t partIndex : parts) {
        if (stopRequested.load() || evaluations >= maxEvaluations || partIndex >= document.parts.size()) {
            break;
        }
        const Pose base = state.poses[partIndex];
        Pose bestPose = base;
        double bestScore = state.totalScore;
        bool found = false;

        for (const FrontierCandidate& frontier : frontiers) {
            if (frontier.sourcePart == partIndex || evaluations >= maxEvaluations || stopRequested.load()) {
                continue;
            }
            const Vec2 anchors[] = {
                frontier.anchor,
                frontier.featureBounds.isValid() ? frontier.featureBounds.min : frontier.anchor,
                frontier.featureBounds.isValid() ? frontier.featureBounds.center() : frontier.anchor
            };
            for (Vec2 anchor : anchors) {
                Pose candidate = poseWithMinAt(document.parts[partIndex], base, anchor);
                if (samePose(candidate, base)) {
                    continue;
                }
                const DeltaMove move{partIndex, base, candidate};
                const DeltaEvaluation trial = evaluateMoveDelta(document, settings, state, cache, move);
                ++evaluations;
                if (stats) {
                    ++stats->evaluatedCandidates;
                }
                if (!trial.valid) {
                    classifyRejected(trial, stats);
                    continue;
                }
                const double rankedScore = trial.totalScore - frontier.priority * 0.005;
                if (rankedScore + 1e-9 < bestScore) {
                    bestScore = rankedScore;
                    bestPose = candidate;
                    found = true;
                }
            }
        }

        if (!found) {
            continue;
        }

        std::vector<Pose> verifiedPoses = state.poses;
        verifiedPoses[partIndex] = bestPose;
        LayoutState verified = scorer.evaluate(document, settings, verifiedPoses, &penalties);
        if (verified.valid() && verified.totalScore + 1e-9 < state.totalScore) {
            const DeltaMove accepted{partIndex, state.poses[partIndex], bestPose};
            state = std::move(verified);
            cache.updateAfterAcceptedMove(document, settings, state, accepted, &penalties);
            if (stats) {
                ++stats->acceptedMoves;
                ++stats->bestUpdates;
                ++stats->regionRepackAccepted;
            }
        }
    }

    return state;
}

} // namespace nest
