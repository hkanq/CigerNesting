#include "engine/frontier_analyzer.h"

#include "core/part.h"
#include <algorithm>
#include <cmath>

namespace nest {
namespace {

double clampDouble(double value, double minValue, double maxValue) {
    return std::max(minValue, std::min(value, maxValue));
}

Vec2 clampToSheet(const Document& document, const EngineSettings& settings, Vec2 point) {
    const double left = document.sheet.origin.x + settings.margin;
    const double right = document.sheet.origin.x + document.sheet.width - settings.margin;
    const double bottom = document.sheet.origin.y + settings.margin;
    const double top = document.sheet.origin.y + document.sheet.height - settings.margin;
    if (right > left) {
        point.x = clampDouble(point.x, left, right);
    }
    if (top > bottom) {
        point.y = clampDouble(point.y, bottom, top);
    }
    return point;
}

bool nearExisting(const std::vector<FrontierCandidate>& candidates, Vec2 anchor, double radius, FrontierCandidateKind kind) {
    for (const FrontierCandidate& candidate : candidates) {
        if (candidate.kind == kind && distance(candidate.anchor, anchor) <= radius) {
            return true;
        }
    }
    return false;
}

void addCandidate(
    std::vector<FrontierCandidate>& candidates,
    const Document& document,
    const EngineSettings& settings,
    Vec2 anchor,
    FrontierCandidateKind kind,
    double priority,
    size_t sourcePart = static_cast<size_t>(-1),
    AABB featureBounds = {}) {
    anchor = clampToSheet(document, settings, anchor);
    const double radius = std::max(0.5, settings.partSpacing * 0.25);
    if (nearExisting(candidates, anchor, radius, kind)) {
        return;
    }
    candidates.push_back({anchor, kind, sourcePart, priority, featureBounds});
}

void addSheetLines(std::vector<FrontierCandidate>& candidates, const Document& document, const EngineSettings& settings) {
    const double left = document.sheet.origin.x + settings.margin;
    const double right = document.sheet.origin.x + document.sheet.width - settings.margin;
    const double bottom = document.sheet.origin.y + settings.margin;
    const double top = document.sheet.origin.y + document.sheet.height - settings.margin;
    const double width = std::max(1.0, right - left);
    const double height = std::max(1.0, top - bottom);
    const AABB usable = AABB::fromMinMax({left, bottom}, {right, top});
    const int divisions = 8;
    for (int i = 0; i <= divisions; ++i) {
        const double tx = static_cast<double>(i) / static_cast<double>(divisions);
        addCandidate(candidates, document, settings, {left + width * tx, bottom}, FrontierCandidateKind::SheetLine, 34.0, static_cast<size_t>(-1), usable);
        addCandidate(candidates, document, settings, {left, bottom + height * tx}, FrontierCandidateKind::SheetLine, 31.0, static_cast<size_t>(-1), usable);
    }
}

void addBoundsEdges(std::vector<FrontierCandidate>& candidates, const Document& document, const EngineSettings& settings, const LayoutState& state, std::vector<double>& xs, std::vector<double>& ys) {
    const size_t count = std::min(document.parts.size(), state.poses.size());
    xs.reserve(count * 4 + 8);
    ys.reserve(count * 4 + 8);
    for (size_t i = 0; i < count; ++i) {
        const AABB box = transformedBounds(document.parts[i], state.poses[i]);
        if (!box.isValid()) {
            continue;
        }
        xs.push_back(box.min.x);
        xs.push_back(box.max.x + settings.partSpacing);
        xs.push_back(box.max.x);
        xs.push_back(std::max(document.sheet.origin.x + settings.margin, box.min.x - settings.partSpacing));
        ys.push_back(box.min.y);
        ys.push_back(box.max.y + settings.partSpacing);
        ys.push_back(box.max.y);
        ys.push_back(std::max(document.sheet.origin.y + settings.margin, box.min.y - settings.partSpacing));

        const Vec2 anchors[] = {
            {box.min.x, box.max.y + settings.partSpacing},
            {box.max.x + settings.partSpacing, box.min.y},
            {box.min.x, box.min.y},
            {box.max.x + settings.partSpacing, box.max.y + settings.partSpacing},
            {box.min.x, box.center().y},
            {box.center().x, box.max.y + settings.partSpacing}
        };
        for (const Vec2& anchor : anchors) {
            addCandidate(candidates, document, settings, anchor, FrontierCandidateKind::BoundsEdge, 28.0, i, box);
        }
    }
}

void uniqueSort(std::vector<double>& values, double eps) {
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end(), [&](double a, double b) {
        return std::abs(a - b) <= eps;
    }), values.end());
}

void addSkylineGrid(
    std::vector<FrontierCandidate>& candidates,
    const Document& document,
    const EngineSettings& settings,
    const LayoutState& state,
    std::vector<double> xs,
    std::vector<double> ys) {
    AABB used;
    const size_t count = std::min(document.parts.size(), state.poses.size());
    for (size_t i = 0; i < count; ++i) {
        used.include(transformedBounds(document.parts[i], state.poses[i]));
    }
    if (!used.isValid()) {
        return;
    }

    xs.push_back(document.sheet.origin.x + settings.margin);
    ys.push_back(document.sheet.origin.y + settings.margin);
    xs.push_back(std::min(document.sheet.origin.x + document.sheet.width - settings.margin, used.max.x));
    ys.push_back(std::min(document.sheet.origin.y + document.sheet.height - settings.margin, used.max.y));
    uniqueSort(xs, std::max(0.25, settings.partSpacing * 0.25));
    uniqueSort(ys, std::max(0.25, settings.partSpacing * 0.25));

    const size_t xLimit = settings.performanceProfile == PerformanceProfile::Maximum ? 48u : settings.performanceProfile == PerformanceProfile::Balanced ? 32u : 20u;
    const size_t yLimit = settings.performanceProfile == PerformanceProfile::Maximum ? 48u : settings.performanceProfile == PerformanceProfile::Balanced ? 32u : 20u;
    if (xs.size() > xLimit) {
        const size_t stride = std::max<size_t>(1, xs.size() / xLimit);
        std::vector<double> reduced;
        for (size_t i = 0; i < xs.size() && reduced.size() < xLimit; i += stride) {
            reduced.push_back(xs[i]);
        }
        xs = std::move(reduced);
    }
    if (ys.size() > yLimit) {
        const size_t stride = std::max<size_t>(1, ys.size() / yLimit);
        std::vector<double> reduced;
        for (size_t i = 0; i < ys.size() && reduced.size() < yLimit; i += stride) {
            reduced.push_back(ys[i]);
        }
        ys = std::move(reduced);
    }

    for (double y : ys) {
        for (double x : xs) {
            const bool insideCurrentUsed = x <= used.max.x + 1e-9 && y <= used.max.y + 1e-9;
            const double priority = insideCurrentUsed ? 42.0 : 18.0;
            addCandidate(candidates, document, settings, {x, y}, FrontierCandidateKind::Skyline, priority, static_cast<size_t>(-1), used);
        }
    }
}

void addEmptyCellCenters(std::vector<FrontierCandidate>& candidates, const Document& document, const EngineSettings& settings, const LayoutState& state) {
    AABB used;
    const size_t count = std::min(document.parts.size(), state.poses.size());
    for (size_t i = 0; i < count; ++i) {
        used.include(transformedBounds(document.parts[i], state.poses[i]));
    }
    if (!used.isValid()) {
        return;
    }

    const int columns = settings.performanceProfile == PerformanceProfile::Maximum ? 8 : 6;
    const int rows = settings.performanceProfile == PerformanceProfile::Maximum ? 6 : 4;
    const double cellW = used.width() / static_cast<double>(columns);
    const double cellH = used.height() / static_cast<double>(rows);
    if (cellW <= 1e-9 || cellH <= 1e-9) {
        return;
    }

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < columns; ++x) {
            AABB cell = AABB::fromMinMax(
                {used.min.x + static_cast<double>(x) * cellW, used.min.y + static_cast<double>(y) * cellH},
                {used.min.x + static_cast<double>(x + 1) * cellW, used.min.y + static_cast<double>(y + 1) * cellH});
            double occupied = 0.0;
            for (size_t i = 0; i < count; ++i) {
                const AABB box = transformedBounds(document.parts[i], state.poses[i]);
                const double ix = std::max(0.0, std::min(cell.max.x, box.max.x) - std::max(cell.min.x, box.min.x));
                const double iy = std::max(0.0, std::min(cell.max.y, box.max.y) - std::max(cell.min.y, box.min.y));
                occupied += ix * iy;
            }
            const double ratio = occupied / std::max(1.0, cell.area());
            if (ratio < 0.22) {
                addCandidate(candidates, document, settings, cell.center(), FrontierCandidateKind::EmptyCell, 38.0 * (1.0 - ratio), static_cast<size_t>(-1), cell);
                addCandidate(candidates, document, settings, cell.min, FrontierCandidateKind::EmptyCell, 34.0 * (1.0 - ratio), static_cast<size_t>(-1), cell);
            }
        }
    }
}

size_t frontierLimit(const EngineSettings& settings, size_t partCount) {
    size_t limit = settings.performanceProfile == PerformanceProfile::Fast ? 192u :
        settings.performanceProfile == PerformanceProfile::Maximum ? 1400u : 720u;
    if (partCount > 300) {
        limit = std::max<size_t>(320, limit / 2);
    } else if (partCount > 100) {
        limit = std::max<size_t>(420, limit * 2 / 3);
    }
    return limit;
}

} // namespace

std::vector<FrontierCandidate> FrontierAnalyzer::analyze(
    const Document& document,
    const EngineSettings& settings,
    const LayoutState& state) const {
    std::vector<FrontierCandidate> candidates;
    candidates.reserve(512);
    std::vector<double> xs;
    std::vector<double> ys;

    addSheetLines(candidates, document, settings);
    addBoundsEdges(candidates, document, settings, state, xs, ys);
    addSkylineGrid(candidates, document, settings, state, std::move(xs), std::move(ys));
    addEmptyCellCenters(candidates, document, settings, state);

    std::stable_sort(candidates.begin(), candidates.end(), [](const FrontierCandidate& a, const FrontierCandidate& b) {
        return a.priority > b.priority;
    });
    const size_t limit = frontierLimit(settings, document.parts.size());
    if (candidates.size() > limit) {
        candidates.resize(limit);
    }
    return candidates;
}

} // namespace nest
