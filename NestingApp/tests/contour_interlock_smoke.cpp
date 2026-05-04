#include "core/document.h"
#include "engine/layout_score.h"
#include "engine/multi_start_solver.h"
#include "engine/penalty_system.h"
#include "geometry/collision.h"
#include "geometry/transformed_shape.h"
#include <atomic>
#include <iostream>
#include <utility>
#include <vector>

namespace {

using namespace nest;

Ring boxRing(double x0, double y0, double x1, double y1) {
    Ring ring;
    ring.points = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}, {x0, y0}};
    return ring;
}

Part partFromRings(std::vector<Ring> rings) {
    Part part;
    part.rings = std::move(rings);
    part.updateDerivedGeometry();
    return part;
}

Part boxPart(double w, double h) {
    return partFromRings({boxRing(0.0, 0.0, w, h)});
}

Part donutPart() {
    Ring hole = boxRing(34.0, 34.0, 76.0, 76.0);
    hole.isHole = true;
    return partFromRings({boxRing(0.0, 0.0, 110.0, 110.0), hole});
}

Part bLikePart() {
    Ring holeA = boxRing(24.0, 18.0, 62.0, 50.0);
    Ring holeB = boxRing(24.0, 76.0, 62.0, 110.0);
    holeA.isHole = true;
    holeB.isHole = true;
    return partFromRings({boxRing(0.0, 0.0, 82.0, 130.0), holeA, holeB});
}

Part cShapePart() {
    Ring ring;
    ring.points = {
        {0.0, 0.0}, {110.0, 0.0}, {110.0, 22.0}, {34.0, 22.0},
        {34.0, 64.0}, {110.0, 64.0}, {110.0, 86.0}, {0.0, 86.0}, {0.0, 0.0}
    };
    return partFromRings({ring});
}

Part notchPart() {
    Ring ring;
    ring.points = {
        {0.0, 0.0}, {120.0, 0.0}, {120.0, 85.0}, {82.0, 85.0},
        {82.0, 34.0}, {42.0, 34.0}, {42.0, 85.0}, {0.0, 85.0}, {0.0, 0.0}
    };
    return partFromRings({ring});
}

EngineSettings settings(double width, double height) {
    EngineSettings s;
    s.sheetWidth = width;
    s.sheetHeight = height;
    s.margin = 6.0;
    s.partSpacing = 0.0;
    s.allowRotation = false;
    s.rotationMode = RotationMode::None;
    s.allowMirroring = true;
    s.performanceProfile = PerformanceProfile::Maximum;
    s.qualityMode = QualityMode::MaxQuality;
    s.timeLimitSeconds = 5.0;
    s.cpuThreadCount = 1;
    s.deterministic = true;
    s.randomSeed = 91u;
    s.collisionTolerance = 0.01;
    s.placementStrategy = PlacementStrategy::CenterOut;
    return s;
}

LayoutState solve(Document& document, EngineSettings s) {
    document.sheet.width = s.sheetWidth;
    document.sheet.height = s.sheetHeight;
    document.sheet.margin = s.margin;
    std::atomic_bool stopRequested{false};
    MultiStartSolver solver;
    LayoutState solved = solver.solve(document, s, stopRequested, {});
    PenaltySystem penalties;
    return LayoutScore{}.evaluate(document, s, solved.poses, &penalties);
}

Vec2 partCenter(const Document& document, const LayoutState& state, size_t index) {
    return transformPart(document.parts[index], state.poses[index], static_cast<int>(index)).bounds.center();
}

bool pointInsideTransformedHole(const Document& document, const LayoutState& state, size_t owner, Vec2 point) {
    const TransformedPart transformed = transformPart(document.parts[owner], state.poses[owner], static_cast<int>(owner));
    for (const TransformedRing& ring : transformed.rings) {
        if (ring.isHole && pointInRing(ring.points, point, 0.01) != PointLocation::Outside) {
            return true;
        }
    }
    return false;
}

bool runDonut() {
    Document doc;
    doc.addPart(donutPart());
    doc.addPart(boxPart(18.0, 18.0));
    LayoutState solved = solve(doc, settings(190.0, 145.0));
    const bool inside = solved.valid() && pointInsideTransformedHole(doc, solved, 0, partCenter(doc, solved, 1));
    std::cout << (inside ? "PASS" : "FAIL") << ": donut hole interlock util=" << solved.utilization << "\n";
    return inside;
}

bool runBLike() {
    Document doc;
    doc.addPart(bLikePart());
    doc.addPart(boxPart(16.0, 16.0));
    doc.addPart(boxPart(15.0, 15.0));
    LayoutState solved = solve(doc, settings(210.0, 165.0));
    const bool first = solved.valid() && pointInsideTransformedHole(doc, solved, 0, partCenter(doc, solved, 1));
    const bool second = solved.valid() && pointInsideTransformedHole(doc, solved, 0, partCenter(doc, solved, 2));
    std::cout << ((first && second) ? "PASS" : "FAIL") << ": B-like holes first=" << first << " second=" << second << "\n";
    return first && second;
}

bool runCShape() {
    Document doc;
    doc.addPart(cShapePart());
    doc.addPart(boxPart(24.0, 24.0));
    LayoutState solved = solve(doc, settings(190.0, 125.0));
    const Vec2 center = partCenter(doc, solved, 1);
    const TransformedPart owner = transformPart(doc.parts[0], solved.poses[0], 0);
    const AABB gap = AABB::fromMinMax(
        {owner.bounds.min.x + 38.0, owner.bounds.min.y + 25.0},
        {owner.bounds.max.x - 8.0, owner.bounds.min.y + 62.0});
    const bool inGap = solved.valid() && center.x >= gap.min.x && center.x <= gap.max.x && center.y >= gap.min.y && center.y <= gap.max.y;
    std::cout << (inGap ? "PASS" : "FAIL") << ": C-shape cavity center=(" << center.x << "," << center.y << ")\n";
    return inGap;
}

bool runNotch() {
    Document doc;
    doc.addPart(notchPart());
    doc.addPart(boxPart(28.0, 28.0));
    LayoutState solved = solve(doc, settings(215.0, 125.0));
    const Vec2 center = partCenter(doc, solved, 1);
    const TransformedPart owner = transformPart(doc.parts[0], solved.poses[0], 0);
    const bool nearNotch = solved.valid() &&
        center.x > owner.bounds.min.x + 42.0 &&
        center.x < owner.bounds.max.x - 42.0 &&
        center.y > owner.bounds.min.y + 34.0;
    std::cout << (nearNotch ? "PASS" : "FAIL") << ": notch contour contact center=(" << center.x << "," << center.y << ")\n";
    return nearNotch;
}

} // namespace

int main() {
    bool ok = true;
    ok = runDonut() && ok;
    ok = runBLike() && ok;
    ok = runCShape() && ok;
    ok = runNotch() && ok;
    return ok ? 0 : 1;
}
