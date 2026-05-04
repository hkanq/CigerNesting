#include "core/document.h"
#include "engine/acceptance.h"
#include "engine/escape_search.h"
#include "engine/gap_filling.h"
#include "engine/layout_score.h"
#include "engine/penalty_system.h"
#include "engine/rearrangement.h"
#include "engine/tabu_memory.h"
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

Pose pose(double x, double y) {
    Pose out;
    out.x = x;
    out.y = y;
    return out;
}

EngineSettings settings(double width, double height, PerformanceProfile profile = PerformanceProfile::Balanced) {
    EngineSettings out;
    out.sheetWidth = width;
    out.sheetHeight = height;
    out.margin = 5.0;
    out.partSpacing = 4.0;
    out.collisionTolerance = 1e-6;
    out.performanceProfile = profile;
    return out;
}

LayoutState evaluate(const Document& document, const EngineSettings& settings, const std::vector<Pose>& poses, PenaltySystem& penalties) {
    return LayoutScore{}.evaluate(document, settings, poses, &penalties);
}

bool expect(const char* name, bool condition) {
    std::cout << (condition ? "PASS: " : "FAIL: ") << name << "\n";
    return condition;
}

Document compactClusterDocument(const EngineSettings& s) {
    Document doc;
    doc.sheet.width = s.sheetWidth;
    doc.sheet.height = s.sheetHeight;
    doc.sheet.margin = s.margin;
    doc.addPart(boxPart(14.0, 14.0));
    doc.addPart(boxPart(14.0, 14.0));
    doc.addPart(boxPart(14.0, 14.0));
    return doc;
}

bool acceptanceProfilesDiffer() {
    EngineSettings fast = settings(200.0, 120.0, PerformanceProfile::Fast);
    EngineSettings maximum = settings(200.0, 120.0, PerformanceProfile::Maximum);
    AcceptanceCriteria fastAcceptance(fast);
    AcceptanceCriteria maxAcceptance(maximum);
    int fastAccepted = 0;
    int maxAccepted = 0;
    for (int i = 0; i < 200; ++i) {
        fastAccepted += fastAcceptance.decide(10000.0, 10100.0, 0, 20, 17u, static_cast<size_t>(i)).accepted ? 1 : 0;
        maxAccepted += maxAcceptance.decide(10000.0, 10100.0, 0, 20, 17u, static_cast<size_t>(i)).accepted ? 1 : 0;
    }
    std::cout << "acceptance fast=" << fastAccepted << " maximum=" << maxAccepted << "\n";
    return maxAccepted > fastAccepted && maxAccepted > 0;
}

bool tabuBlocksRepeats() {
    TabuMemory tabu(4);
    const Pose a = pose(10.0, 10.0);
    const Pose b = pose(40.0, 10.0);
    std::vector<Pose> layout{a, b};
    tabu.rememberMove(0, a, b);
    tabu.rememberLayout(layout);
    return tabu.containsMove(0, a, b) && tabu.containsLayout(layout);
}

bool escapeAcceptsTemporaryWorseMove() {
    EngineSettings s = settings(240.0, 100.0, PerformanceProfile::Maximum);
    Document doc = compactClusterDocument(s);
    PenaltySystem penalties;
    std::vector<Pose> poses{pose(10.0, 30.0), pose(28.0, 30.0), pose(46.0, 30.0)};
    LayoutState before = evaluate(doc, s, poses, penalties);
    std::atomic_bool stop{false};
    SolverStats stats;
    LayoutState escaped = EscapeSearch{}.escape(doc, s, before, penalties, stop, 1234u, &stats);
    std::cout << "escape beforeScore=" << before.totalScore
              << " escapedScore=" << escaped.totalScore
              << " escapeAttempts=" << stats.escapeAttempts
              << " escapeAccepted=" << stats.escapeAccepted
              << " acceptedWorse=" << stats.acceptedWorseMoves << "\n";
    return before.valid() && escaped.valid() && stats.escapeAccepted > 0 && stats.acceptedWorseMoves > 0 && escaped.totalScore > before.totalScore;
}

bool worseThenBetterRecovery() {
    EngineSettings s = settings(240.0, 100.0, PerformanceProfile::Maximum);
    Document doc = compactClusterDocument(s);
    PenaltySystem penalties;
    std::vector<Pose> poses{pose(10.0, 30.0), pose(28.0, 30.0), pose(46.0, 30.0)};
    LayoutState before = evaluate(doc, s, poses, penalties);
    std::atomic_bool stop{false};
    SolverStats escapeStats;
    LayoutState escaped = EscapeSearch{}.escape(doc, s, before, penalties, stop, 5678u, &escapeStats);
    SolverStats rearrangeStats;
    LayoutState recovered = Rearrangement{}.improve(doc, s, escaped, penalties, stop, &rearrangeStats);
    std::cout << "recovery before=" << before.totalScore
              << " escaped=" << escaped.totalScore
              << " recovered=" << recovered.totalScore
              << " rearrangeBestUpdates=" << rearrangeStats.bestUpdates << "\n";
    return escaped.valid() && recovered.valid() && escaped.totalScore > before.totalScore && recovered.totalScore < escaped.totalScore;
}

bool adaptiveStackProducesSeveralBestUpdates() {
    EngineSettings s = settings(190.0, 80.0, PerformanceProfile::Maximum);
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
    LayoutState state = evaluate(doc, s, poses, penalties);
    SolverStats stats;
    state = GapFilling{}.fillGaps(doc, s, state, penalties, stop, &stats);
    state = Rearrangement{}.improve(doc, s, state, penalties, stop, &stats);
    std::cout << "adaptive_stack score=" << state.totalScore
              << " bestUpdates=" << stats.bestUpdates
              << " acceptedWorse=" << stats.acceptedWorseMoves << "\n";
    return state.valid() && stats.bestUpdates > 3;
}

} // namespace

int main() {
    bool ok = true;
    ok = expect("annealing acceptance is profile-sensitive", acceptanceProfilesDiffer()) && ok;
    ok = expect("tabu memory blocks repeated move/layout", tabuBlocksRepeats()) && ok;
    ok = expect("escape accepts a temporary worse valid layout", escapeAcceptsTemporaryWorseMove()) && ok;
    ok = expect("escape can be followed by better recovery", worseThenBetterRecovery()) && ok;
    ok = expect("adaptive stack produces bestUpdates > 3", adaptiveStackProducesSeveralBestUpdates()) && ok;
    return ok ? 0 : 1;
}
