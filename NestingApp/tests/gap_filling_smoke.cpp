#include "core/document.h"
#include "engine/gap_filling.h"
#include "engine/layout_score.h"
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

Part boxPart(double width, double height) {
    return partFromRings({boxRing(0.0, 0.0, width, height)});
}

Part donutPart() {
    Ring hole = boxRing(30.0, 30.0, 70.0, 70.0);
    hole.isHole = true;
    return partFromRings({boxRing(0.0, 0.0, 100.0, 100.0), hole});
}

Part bLikePart() {
    Ring holeA = boxRing(25.0, 20.0, 65.0, 55.0);
    Ring holeB = boxRing(25.0, 85.0, 65.0, 120.0);
    holeA.isHole = true;
    holeB.isHole = true;
    return partFromRings({boxRing(0.0, 0.0, 85.0, 140.0), holeA, holeB});
}

Part cShapePart() {
    Ring ring;
    ring.points = {
        {0.0, 0.0}, {100.0, 0.0}, {100.0, 25.0}, {35.0, 25.0},
        {35.0, 75.0}, {100.0, 75.0}, {100.0, 100.0}, {0.0, 100.0}, {0.0, 0.0}
    };
    return partFromRings({ring});
}

Pose pose(double x, double y) {
    Pose out;
    out.x = x;
    out.y = y;
    return out;
}

EngineSettings baseSettings(double width, double height) {
    EngineSettings settings;
    settings.sheetWidth = width;
    settings.sheetHeight = height;
    settings.margin = 5.0;
    settings.partSpacing = 4.0;
    settings.collisionTolerance = 1e-6;
    settings.performanceProfile = PerformanceProfile::Balanced;
    return settings;
}

LayoutState evaluate(const Document& document, const EngineSettings& settings, const std::vector<Pose>& poses, PenaltySystem& penalties) {
    return LayoutScore{}.evaluate(document, settings, poses, &penalties);
}

bool expect(const char* name, bool condition) {
    std::cout << (condition ? "PASS: " : "FAIL: ") << name << "\n";
    return condition;
}

bool partCenterInSourceHole(const Document& document, const LayoutState& state, size_t movingPart, size_t sourcePart) {
    if (movingPart >= document.parts.size() || movingPart >= state.poses.size() ||
        sourcePart >= document.parts.size() || sourcePart >= state.poses.size()) {
        return false;
    }
    const TransformedPart moved = transformPart(document.parts[movingPart], state.poses[movingPart], static_cast<int>(movingPart));
    const TransformedPart source = transformPart(document.parts[sourcePart], state.poses[sourcePart], static_cast<int>(sourcePart));
    const Vec2 center = moved.bounds.center();
    for (const TransformedRing& ring : source.rings) {
        if (ring.isHole && pointInRing(ring.points, center, 1e-6) != PointLocation::Outside) {
            return true;
        }
    }
    return false;
}

bool partCenterInAnySourceHole(const Document& document, const LayoutState& state, size_t movingPart, size_t sourcePart) {
    return partCenterInSourceHole(document, state, movingPart, sourcePart);
}

LayoutState runGapFilling(const Document& document, const EngineSettings& settings, LayoutState state, SolverStats* stats = nullptr) {
    PenaltySystem penalties;
    std::atomic_bool stop{false};
    GapFilling gapFilling;
    return gapFilling.fillGaps(document, settings, std::move(state), penalties, stop, stats);
}

bool donutHoleFilled() {
    EngineSettings settings = baseSettings(230.0, 150.0);
    Document doc;
    doc.sheet.width = settings.sheetWidth;
    doc.sheet.height = settings.sheetHeight;
    doc.sheet.margin = settings.margin;
    doc.addPart(donutPart());
    doc.addPart(boxPart(20.0, 20.0));

    PenaltySystem penalties;
    std::vector<Pose> poses{pose(20.0, 20.0), pose(170.0, 20.0)};
    LayoutState before = evaluate(doc, settings, poses, penalties);
    SolverStats stats;
    LayoutState after = runGapFilling(doc, settings, before, &stats);
    std::cout << "donut beforeScore=" << before.totalScore
              << " afterScore=" << after.totalScore
              << " accepted=" << stats.acceptedMoves << "\n";
    return after.valid() && after.totalScore < before.totalScore && partCenterInAnySourceHole(doc, after, 1, 0);
}

bool bLikeHolesFilled() {
    EngineSettings settings = baseSettings(260.0, 180.0);
    Document doc;
    doc.sheet.width = settings.sheetWidth;
    doc.sheet.height = settings.sheetHeight;
    doc.sheet.margin = settings.margin;
    doc.addPart(bLikePart());
    doc.addPart(boxPart(16.0, 16.0));
    doc.addPart(boxPart(16.0, 16.0));

    PenaltySystem penalties;
    std::vector<Pose> poses{pose(15.0, 15.0), pose(190.0, 20.0), pose(214.0, 20.0)};
    LayoutState before = evaluate(doc, settings, poses, penalties);
    SolverStats stats;
    LayoutState after = runGapFilling(doc, settings, before, &stats);
    const bool first = partCenterInAnySourceHole(doc, after, 1, 0);
    const bool second = partCenterInAnySourceHole(doc, after, 2, 0);
    std::cout << "b_like beforeScore=" << before.totalScore
              << " afterScore=" << after.totalScore
              << " accepted=" << stats.acceptedMoves
              << " firstHole=" << first
              << " secondHole=" << second << "\n";
    return after.valid() && after.totalScore < before.totalScore && first && second;
}

bool concavityFilled() {
    EngineSettings settings = baseSettings(230.0, 150.0);
    Document doc;
    doc.sheet.width = settings.sheetWidth;
    doc.sheet.height = settings.sheetHeight;
    doc.sheet.margin = settings.margin;
    doc.addPart(cShapePart());
    doc.addPart(boxPart(20.0, 20.0));

    PenaltySystem penalties;
    std::vector<Pose> poses{pose(20.0, 20.0), pose(170.0, 20.0)};
    LayoutState before = evaluate(doc, settings, poses, penalties);
    SolverStats stats;
    LayoutState after = runGapFilling(doc, settings, before, &stats);
    const AABB small = transformedBounds(doc.parts[1], after.poses[1]);
    const Vec2 center = small.center();
    const bool inNotch = center.x > 55.0 && center.x < 120.0 && center.y > 45.0 && center.y < 90.0;
    std::cout << "c_shape beforeScore=" << before.totalScore
              << " afterScore=" << after.totalScore
              << " accepted=" << stats.acceptedMoves
              << " center=(" << center.x << "," << center.y << ")\n";
    return after.valid() && after.totalScore < before.totalScore && inNotch;
}

bool validityAfterGapFilling() {
    EngineSettings settings = baseSettings(260.0, 180.0);
    Document doc;
    doc.sheet.width = settings.sheetWidth;
    doc.sheet.height = settings.sheetHeight;
    doc.sheet.margin = settings.margin;
    doc.addPart(donutPart());
    doc.addPart(boxPart(18.0, 18.0));
    doc.addPart(boxPart(18.0, 18.0));

    PenaltySystem penalties;
    std::vector<Pose> poses{pose(20.0, 20.0), pose(175.0, 20.0), pose(198.0, 20.0)};
    LayoutState before = evaluate(doc, settings, poses, penalties);
    LayoutState after = runGapFilling(doc, settings, before);
    std::cout << "validity collisions=" << after.collisionCount
              << " invalid=" << after.invalidPartCount
              << " spacingPenalty=" << after.spacingPenalty << "\n";
    return after.valid();
}

bool hundredPartBestUpdates() {
    EngineSettings settings = baseSettings(900.0, 230.0);
    settings.performanceProfile = PerformanceProfile::Maximum;
    Document doc;
    doc.sheet.width = settings.sheetWidth;
    doc.sheet.height = settings.sheetHeight;
    doc.sheet.margin = settings.margin;
    for (size_t i = 0; i < 5; ++i) {
        doc.addPart(donutPart());
    }
    for (size_t i = 5; i < 100; ++i) {
        doc.addPart(boxPart(12.0, 12.0));
    }

    std::vector<Pose> poses(doc.parts.size());
    for (size_t i = 0; i < 5; ++i) {
        poses[i] = pose(10.0 + static_cast<double>(i) * 120.0, 10.0);
    }
    double x = 10.0;
    double y = 135.0;
    for (size_t i = 5; i < poses.size(); ++i) {
        poses[i] = pose(x, y);
        x += 18.0;
        if (x > 860.0) {
            x = 10.0;
            y += 18.0;
        }
    }

    PenaltySystem penalties;
    LayoutState before = evaluate(doc, settings, poses, penalties);
    SolverStats stats;
    LayoutState after = runGapFilling(doc, settings, before, &stats);
    std::cout << "100_part beforeScore=" << before.totalScore
              << " afterScore=" << after.totalScore
              << " accepted=" << stats.acceptedMoves
              << " bestUpdates=" << stats.bestUpdates
              << " evaluated=" << stats.evaluatedCandidates << "\n";
    return after.valid() && stats.bestUpdates > 1 && after.totalScore < before.totalScore;
}

} // namespace

int main() {
    bool ok = true;
    ok = expect("donut + small square uses hole", donutHoleFilled()) && ok;
    ok = expect("B-like two holes accept two small parts", bLikeHolesFilled()) && ok;
    ok = expect("concave C-shape indentation accepts small part", concavityFilled()) && ok;
    ok = expect("gap filling keeps collision/invalid/spacing at zero", validityAfterGapFilling()) && ok;
    ok = expect("100 part gap filling produces multiple best updates", hundredPartBestUpdates()) && ok;
    return ok ? 0 : 1;
}
