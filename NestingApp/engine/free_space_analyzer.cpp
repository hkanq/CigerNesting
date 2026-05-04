#include "engine/free_space_analyzer.h"

#include "core/part.h"
#include "geometry/transformed_shape.h"
#include <algorithm>
#include <cmath>

namespace nest {
namespace {

double polygonSignedArea(const std::vector<Vec2>& points) {
    const size_t count = points.size();
    if (count < 3) {
        return 0.0;
    }
    const bool closed = almostEqual(points.front(), points.back(), 1e-9);
    const size_t usable = closed ? count - 1 : count;
    if (usable < 3) {
        return 0.0;
    }
    double sum = 0.0;
    for (size_t i = 0; i < usable; ++i) {
        const Vec2& a = points[i];
        const Vec2& b = points[(i + 1) % usable];
        sum += a.x * b.y - b.x * a.y;
    }
    return sum * 0.5;
}

double clampDouble(double value, double minValue, double maxValue) {
    return std::max(minValue, std::min(value, maxValue));
}

Vec2 clampedToUsableSheet(const Document& document, const EngineSettings& settings, Vec2 point) {
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

void pushCandidate(
    std::vector<FreeSpaceCandidate>& candidates,
    const Document& document,
    const EngineSettings& settings,
    Vec2 anchor,
    FreeSpaceCandidateKind kind,
    double priority,
    size_t sourcePart = static_cast<size_t>(-1),
    int sourceRing = -1,
    AABB featureBounds = {}) {
    anchor = clampedToUsableSheet(document, settings, anchor);
    const double uniqueRadius = std::max(0.5, settings.partSpacing * 0.20);
    for (const FreeSpaceCandidate& existing : candidates) {
        if (distance(existing.anchor, anchor) <= uniqueRadius && existing.kind == kind) {
            return;
        }
    }
    candidates.push_back({anchor, kind, sourcePart, sourceRing, priority, featureBounds});
}

void addBoxAnchors(
    std::vector<FreeSpaceCandidate>& candidates,
    const Document& document,
    const EngineSettings& settings,
    const AABB& box,
    FreeSpaceCandidateKind kind,
    double priority,
    double inset,
    size_t sourcePart = static_cast<size_t>(-1),
    int sourceRing = -1) {
    if (!box.isValid()) {
        return;
    }
    const AABB adjusted = AABB::fromMinMax(
        {box.min.x + inset, box.min.y + inset},
        {box.max.x - inset, box.max.y - inset});
    const AABB usable = adjusted.isValid() ? adjusted : box;
    const Vec2 center = usable.center();
    const Vec2 anchors[] = {
        center,
        {usable.min.x, usable.min.y},
        {usable.max.x, usable.min.y},
        {usable.max.x, usable.max.y},
        {usable.min.x, usable.max.y},
        {center.x, usable.min.y},
        {usable.max.x, center.y},
        {center.x, usable.max.y},
        {usable.min.x, center.y}
    };
    for (const Vec2& anchor : anchors) {
        pushCandidate(candidates, document, settings, anchor, kind, priority, sourcePart, sourceRing, box);
    }
}

void addSheetCandidates(std::vector<FreeSpaceCandidate>& candidates, const Document& document, const EngineSettings& settings) {
    AABB usable = AABB::fromMinMax(
        {document.sheet.origin.x + settings.margin, document.sheet.origin.y + settings.margin},
        {document.sheet.origin.x + document.sheet.width - settings.margin, document.sheet.origin.y + document.sheet.height - settings.margin});
    if (document.sheet.hasCustomProfile() && !document.sheet.profile().outerContour.points.empty()) {
        AABB custom;
        for (const Vec2& point : document.sheet.profile().outerContour.points) {
            custom.include(point);
        }
        usable.include(custom);
    }
    addBoxAnchors(candidates, document, settings, usable, FreeSpaceCandidateKind::SheetCorner, 35.0, 0.0);
    const Vec2 center = usable.center();
    const Vec2 boundary[] = {
        {center.x, usable.min.y + settings.margin},
        {usable.max.x - settings.margin, center.y},
        {center.x, usable.max.y - settings.margin},
        {usable.min.x + settings.margin, center.y}
    };
    for (const Vec2& anchor : boundary) {
        pushCandidate(candidates, document, settings, anchor, FreeSpaceCandidateKind::SheetBoundary, 26.0, static_cast<size_t>(-1), -1, usable);
    }
}

void addPartOuterCandidates(
    std::vector<FreeSpaceCandidate>& candidates,
    const Document& document,
    const EngineSettings& settings,
    const TransformedRing& ring,
    size_t partIndex,
    int ringIndex) {
    if (!ring.bounds.isValid()) {
        return;
    }
    const double clearance = std::max(1.0, settings.partSpacing);
    const AABB expanded = ring.bounds.expanded(clearance);
    const Vec2 center = ring.bounds.center();
    const Vec2 anchors[] = {
        {expanded.min.x, center.y},
        {expanded.max.x, center.y},
        {center.x, expanded.min.y},
        {center.x, expanded.max.y},
        {expanded.min.x, expanded.min.y},
        {expanded.max.x, expanded.min.y},
        {expanded.max.x, expanded.max.y},
        {expanded.min.x, expanded.max.y}
    };
    for (const Vec2& anchor : anchors) {
        pushCandidate(candidates, document, settings, anchor, FreeSpaceCandidateKind::PartOuter, 18.0, partIndex, ringIndex, ring.bounds);
    }
}

void addHoleCandidates(
    std::vector<FreeSpaceCandidate>& candidates,
    const Document& document,
    const EngineSettings& settings,
    const TransformedRing& ring,
    size_t partIndex,
    int ringIndex) {
    const double inset = std::max(0.0, settings.partSpacing);
    addBoxAnchors(candidates, document, settings, ring.bounds, FreeSpaceCandidateKind::PartHole, 85.0, inset, partIndex, ringIndex);
}

void addConcavityCandidates(
    std::vector<FreeSpaceCandidate>& candidates,
    const Document& document,
    const EngineSettings& settings,
    const TransformedRing& ring,
    size_t partIndex,
    int ringIndex) {
    const std::vector<Vec2>& points = ring.points;
    if (points.size() < 4) {
        return;
    }
    const bool closed = almostEqual(points.front(), points.back(), 1e-9);
    const size_t usable = closed ? points.size() - 1 : points.size();
    if (usable < 4) {
        return;
    }
    const double area = polygonSignedArea(points);
    if (std::abs(area) < 1e-9) {
        return;
    }
    const double orientation = area > 0.0 ? 1.0 : -1.0;
    const double nudge = std::max(1.0, settings.partSpacing + 2.0);
    std::vector<Vec2> reflexPoints;
    for (size_t i = 0; i < usable; ++i) {
        const Vec2 prev = points[(i + usable - 1) % usable];
        const Vec2 current = points[i];
        const Vec2 next = points[(i + 1) % usable];
        const double turn = cross(current - prev, next - current);
        if (orientation * turn >= -1e-6) {
            continue;
        }
        reflexPoints.push_back(current);
        Vec2 bisector = (prev - current) + (next - current);
        const double length = bisector.length();
        if (length > 1e-9) {
            bisector = bisector / length;
        }
        pushCandidate(candidates, document, settings, current, FreeSpaceCandidateKind::Concavity, 65.0, partIndex, ringIndex, ring.bounds);
        pushCandidate(candidates, document, settings, current + bisector * nudge, FreeSpaceCandidateKind::Concavity, 68.0, partIndex, ringIndex, ring.bounds);
        pushCandidate(candidates, document, settings, (prev + current + next) / 3.0, FreeSpaceCandidateKind::Concavity, 58.0, partIndex, ringIndex, ring.bounds);
    }

    for (size_t a = 0; a < reflexPoints.size(); ++a) {
        for (size_t b = a + 1; b < reflexPoints.size(); ++b) {
            const Vec2 p = reflexPoints[a];
            const Vec2 q = reflexPoints[b];
            if (std::abs(p.x - q.x) <= 1e-6) {
                const double openSide = std::abs(ring.bounds.max.x - p.x) > std::abs(p.x - ring.bounds.min.x)
                    ? ring.bounds.max.x
                    : ring.bounds.min.x;
                pushCandidate(
                    candidates,
                    document,
                    settings,
                    {(p.x + openSide) * 0.5, (p.y + q.y) * 0.5},
                    FreeSpaceCandidateKind::Concavity,
                    92.0,
                    partIndex,
                    ringIndex,
                    AABB::fromMinMax({std::min(p.x, openSide), std::min(p.y, q.y)}, {std::max(p.x, openSide), std::max(p.y, q.y)}));
            }
            if (std::abs(p.y - q.y) <= 1e-6) {
                const double openSide = std::abs(ring.bounds.max.y - p.y) > std::abs(p.y - ring.bounds.min.y)
                    ? ring.bounds.max.y
                    : ring.bounds.min.y;
                pushCandidate(
                    candidates,
                    document,
                    settings,
                    {(p.x + q.x) * 0.5, (p.y + openSide) * 0.5},
                    FreeSpaceCandidateKind::Concavity,
                    92.0,
                    partIndex,
                    ringIndex,
                    AABB::fromMinMax({std::min(p.x, q.x), std::min(p.y, openSide)}, {std::max(p.x, q.x), std::max(p.y, openSide)}));
            }
        }
    }
}

void addUsedBoundsCandidates(std::vector<FreeSpaceCandidate>& candidates, const Document& document, const EngineSettings& settings, const AABB& used) {
    if (!used.isValid()) {
        return;
    }
    const Vec2 center = used.center();
    const Vec2 anchors[] = {
        center,
        {(used.min.x + center.x) * 0.5, (used.min.y + center.y) * 0.5},
        {(used.max.x + center.x) * 0.5, (used.min.y + center.y) * 0.5},
        {(used.max.x + center.x) * 0.5, (used.max.y + center.y) * 0.5},
        {(used.min.x + center.x) * 0.5, (used.max.y + center.y) * 0.5}
    };
    for (const Vec2& anchor : anchors) {
        pushCandidate(candidates, document, settings, anchor, FreeSpaceCandidateKind::UsedBoundsGap, 32.0, static_cast<size_t>(-1), -1, used);
    }
}

void addForbiddenZoneCandidates(std::vector<FreeSpaceCandidate>& candidates, const Document& document, const EngineSettings& settings) {
    const double clearance = std::max(1.0, settings.margin + settings.partSpacing);
    for (const Ring& zone : document.sheet.profile().forbiddenZones) {
        AABB bounds;
        for (const Vec2& point : zone.points) {
            bounds.include(point);
        }
        if (!bounds.isValid()) {
            continue;
        }
        const AABB expanded = bounds.expanded(clearance);
        const Vec2 center = bounds.center();
        const Vec2 anchors[] = {
            {expanded.min.x, center.y},
            {expanded.max.x, center.y},
            {center.x, expanded.min.y},
            {center.x, expanded.max.y},
            {expanded.min.x, expanded.min.y},
            {expanded.max.x, expanded.max.y}
        };
        for (const Vec2& anchor : anchors) {
            pushCandidate(candidates, document, settings, anchor, FreeSpaceCandidateKind::ForbiddenZone, 22.0, static_cast<size_t>(-1), -1, bounds);
        }
    }
}

size_t candidateLimit(const EngineSettings& settings, size_t partCount) {
    size_t limit = 384;
    if (settings.performanceProfile == PerformanceProfile::Fast) {
        limit = 128;
    } else if (settings.performanceProfile == PerformanceProfile::Maximum) {
        limit = 768;
    }
    if (partCount > 300) {
        limit = std::max<size_t>(96, limit / 2);
    } else if (partCount > 100) {
        limit = std::max<size_t>(128, limit * 2 / 3);
    }
    return limit;
}

} // namespace

std::vector<FreeSpaceCandidate> FreeSpaceAnalyzer::analyze(
    const Document& document,
    const EngineSettings& settings,
    const LayoutState& state) const {
    std::vector<FreeSpaceCandidate> candidates;
    candidates.reserve(256);
    addSheetCandidates(candidates, document, settings);

    AABB usedBounds;
    const size_t count = std::min(document.parts.size(), state.poses.size());
    for (size_t partIndex = 0; partIndex < count; ++partIndex) {
        const TransformedPart transformed = transformPart(document.parts[partIndex], state.poses[partIndex], static_cast<int>(partIndex));
        usedBounds.include(transformed.bounds);
        for (size_t ringIndex = 0; ringIndex < transformed.rings.size(); ++ringIndex) {
            const TransformedRing& ring = transformed.rings[ringIndex];
            if (ring.isHole) {
                addHoleCandidates(candidates, document, settings, ring, partIndex, static_cast<int>(ringIndex));
            } else {
                addPartOuterCandidates(candidates, document, settings, ring, partIndex, static_cast<int>(ringIndex));
                addConcavityCandidates(candidates, document, settings, ring, partIndex, static_cast<int>(ringIndex));
            }
        }
    }

    addUsedBoundsCandidates(candidates, document, settings, usedBounds);
    addForbiddenZoneCandidates(candidates, document, settings);

    std::stable_sort(candidates.begin(), candidates.end(), [](const FreeSpaceCandidate& a, const FreeSpaceCandidate& b) {
        return a.priority > b.priority;
    });
    const size_t limit = candidateLimit(settings, document.parts.size());
    if (candidates.size() > limit) {
        candidates.resize(limit);
    }
    return candidates;
}

} // namespace nest
