#include "core/document.h"
#include "engine/gap_filling.h"
#include "engine/layout_score.h"
#include "engine/penalty_system.h"
#include "engine/rearrangement.h"
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
    Ring hole = boxRing(30.0, 30.0, 76.0, 76.0);
    hole.isHole = true;
    return partFromRings({boxRing(0.0, 0.0, 110.0, 110.0), hole});
}

Part cShapePart() {
    Ring ring;
    ring.points = {
        {0.0, 0.0}, {110.0, 0.0}, {110.0, 25.0}, {38.0, 25.0},
        {38.0, 82.0}, {110.0, 82.0}, {110.0, 110.0}, {0.0, 110.0}, {0.0, 0.0}
    };
    return partFromRings({ring});
}

Pose pose(double x, double y) {
    Pose out;
    out.x = x;
    out.y = y;
    return out;
}

EngineSettings settings(double width, double height) {
    EngineSettings out;
    out.sheetWidth = width;
    out.sheetHeight = height;
    out.margin = 5.0;
    out.partSpacing = 4.0;
    out.collisionTolerance = 1e-6;
    out.performanceProfile = PerformanceProfile::Balanced;
    return out;
}

LayoutState evaluate(const Document& document, const EngineSettings& settings, const std::vector<Pose>& poses, PenaltySystem& penalties) {
    return LayoutScore{}.evaluate(document, settings, poses, &penalties);
}

LayoutState rearrange(const Document& document, const EngineSettings& settings, LayoutState state, SolverStats& stats) {
    PenaltySystem penalties;
    std::atomic_bool stop{false};
    return Rearrangement{}.improve(document, settings, std::move(state), penalties, stop, &stats);
}

bool expect(const char* name, bool condition) {
    std::cout << (condition ? "PASS: " : "FAIL: ") << name << "\n";
    return condition;
}

bool swapImprovesLayout() {
    EngineSettings s = settings(190.0, 80.0);
    Document doc;
    doc.sheet.width = s.sheetWidth;
    doc.sheet.height = s.sheetHeight;
    doc.sheet.margin = s.margin;
    doc.addPart(boxPart(10.0, 10.0));
    doc.addPart(boxPart(42.0, 18.0));
    doc.addPart(boxPart(12.0, 12.0));

    PenaltySystem penalties;
    std::vector<Pose> poses{pose(10.0, 10.0), pose(125.0, 10.0), pose(10.0, 36.0)};
    LayoutState before = evaluate(doc, s, poses, penalties);
    SolverStats stats;
    LayoutState after = rearrange(doc, s, before, stats);
    std::cout << "swap beforeScore=" << before.totalScore
              << " afterScore=" << after.totalScore
              << " swapAttempts=" << stats.swapAttempts
              << " swapAccepted=" << stats.swapAccepted << "\n";
    return after.valid() && after.totalScore < before.totalScore && stats.swapAccepted > 0;
}

bool donutEjectionChainImproves() {
    EngineSettings s = settings(260.0, 165.0);
    s.performanceProfile = PerformanceProfile::Maximum;
    Document doc;
    doc.sheet.width = s.sheetWidth;
    doc.sheet.height = s.sheetHeight;
    doc.sheet.margin = s.margin;
    doc.addPart(donutPart());
    doc.addPart(boxPart(20.0, 20.0));
    doc.addPart(boxPart(8.0, 8.0));

    PenaltySystem penalties;
    std::vector<Pose> poses{pose(18.0, 18.0), pose(205.0, 24.0), pose(68.0, 68.0)};
    LayoutState before = evaluate(doc, s, poses, penalties);
    SolverStats stats;
    LayoutState after = rearrange(doc, s, before, stats);
    std::cout << "donut_chain beforeScore=" << before.totalScore
              << " afterScore=" << after.totalScore
              << " chainAttempts=" << stats.chainAttempts
              << " chainAccepted=" << stats.chainAccepted << "\n";
    return after.valid() && after.totalScore < before.totalScore && stats.chainAccepted > 0;
}

bool concaveEjectionChainImproves() {
    EngineSettings s = settings(260.0, 165.0);
    s.performanceProfile = PerformanceProfile::Maximum;
    Document doc;
    doc.sheet.width = s.sheetWidth;
    doc.sheet.height = s.sheetHeight;
    doc.sheet.margin = s.margin;
    doc.addPart(cShapePart());
    doc.addPart(boxPart(22.0, 22.0));
    doc.addPart(boxPart(8.0, 8.0));

    PenaltySystem penalties;
    std::vector<Pose> poses{pose(16.0, 16.0), pose(205.0, 24.0), pose(82.0, 60.0)};
    LayoutState before = evaluate(doc, s, poses, penalties);
    SolverStats stats;
    LayoutState after = rearrange(doc, s, before, stats);
    std::cout << "concave_chain beforeScore=" << before.totalScore
              << " afterScore=" << after.totalScore
              << " chainAttempts=" << stats.chainAttempts
              << " chainAccepted=" << stats.chainAccepted << "\n";
    return after.valid() && after.totalScore < before.totalScore && stats.chainAccepted > 0;
}

bool clusterCompactionReducesUsedArea() {
    EngineSettings s = settings(260.0, 100.0);
    Document doc;
    doc.sheet.width = s.sheetWidth;
    doc.sheet.height = s.sheetHeight;
    doc.sheet.margin = s.margin;
    doc.addPart(boxPart(14.0, 14.0));
    doc.addPart(boxPart(14.0, 14.0));
    doc.addPart(boxPart(14.0, 14.0));

    PenaltySystem penalties;
    std::vector<Pose> poses{pose(120.0, 30.0), pose(142.0, 30.0), pose(164.0, 30.0)};
    LayoutState before = evaluate(doc, s, poses, penalties);
    SolverStats stats;
    LayoutState after = rearrange(doc, s, before, stats);
    std::cout << "cluster beforeWidth=" << before.usedWidth
              << " afterWidth=" << after.usedWidth
              << " clusterAttempts=" << stats.clusterAttempts
              << " clusterAccepted=" << stats.clusterAccepted << "\n";
    return after.valid() && after.usedWidth < before.usedWidth && stats.clusterAccepted > 0;
}

bool rearrangementAddsBestUpdateAfterGapFillingStalls() {
    EngineSettings s = settings(190.0, 80.0);
    Document doc;
    doc.sheet.width = s.sheetWidth;
    doc.sheet.height = s.sheetHeight;
    doc.sheet.margin = s.margin;
    doc.addPart(boxPart(10.0, 10.0));
    doc.addPart(boxPart(42.0, 18.0));
    doc.addPart(boxPart(12.0, 12.0));

    PenaltySystem penalties;
    std::vector<Pose> poses{pose(10.0, 10.0), pose(125.0, 10.0), pose(10.0, 36.0)};
    std::atomic_bool stop{false};
    LayoutState before = evaluate(doc, s, poses, penalties);
    SolverStats gapStats;
    LayoutState afterGap = GapFilling{}.fillGaps(doc, s, before, penalties, stop, &gapStats);
    SolverStats rearrangeStats;
    LayoutState after = Rearrangement{}.improve(doc, s, afterGap, penalties, stop, &rearrangeStats);
    std::cout << "gap_bestUpdates=" << gapStats.bestUpdates
              << " rearrange_bestUpdates=" << rearrangeStats.bestUpdates
              << " beforeScore=" << before.totalScore
              << " afterGapScore=" << afterGap.totalScore
              << " afterScore=" << after.totalScore << "\n";
    return after.valid() && gapStats.bestUpdates <= 2 && rearrangeStats.bestUpdates > 0 && after.totalScore < afterGap.totalScore;
}

} // namespace

int main() {
    bool ok = true;
    ok = expect("swap move improves a layout", swapImprovesLayout()) && ok;
    ok = expect("donut hole ejection-chain improves layout", donutEjectionChainImproves()) && ok;
    ok = expect("concave gap ejection-chain improves layout", concaveEjectionChainImproves()) && ok;
    ok = expect("cluster compaction reduces used area", clusterCompactionReducesUsedArea()) && ok;
    ok = expect("rearrangement adds best update after gap filling stalls", rearrangementAddsBestUpdateAfterGapFillingStalls()) && ok;
    return ok ? 0 : 1;
}
