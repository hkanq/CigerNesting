#include "core/document.h"
#include "engine/layout_score.h"
#include "engine/multi_start_solver.h"
#include "engine/nesting_engine.h"
#include "engine/penalty_system.h"
#include "geometry/collision.h"
#include "geometry/transformed_shape.h"
#include "import/importer.h"
#include <atomic>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>
#include <utility>
#include <vector>

namespace {

using namespace nest;
using Clock = std::chrono::steady_clock;

int fail(const char* message) {
    std::cout << "FAIL: " << message << "\n";
    return 1;
}

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

Document makeAdaptiveDocument() {
    Document document;
    document.sheet.width = 420.0;
    document.sheet.height = 300.0;
    document.sheet.margin = 6.0;
    document.addPart(donutPart());
    document.addPart(cShapePart());
    document.addPart(bLikePart());
    document.addPart(boxPart(18.0, 18.0));
    document.addPart(boxPart(16.0, 16.0));
    document.addPart(boxPart(42.0, 12.0));
    document.addPart(boxPart(12.0, 44.0));
    for (int i = 0; i < 10; ++i) {
        document.addPart(boxPart(9.0 + static_cast<double>(i % 4), 8.0 + static_cast<double>(i % 3)));
    }
    return document;
}

EngineSettings adaptiveSettings() {
    EngineSettings settings;
    settings.sheetWidth = 420.0;
    settings.sheetHeight = 300.0;
    settings.margin = 6.0;
    settings.partSpacing = 0.0;
    settings.allowRotation = true;
    settings.rotationMode = RotationMode::ContinuousRefine;
    settings.rotationStepDegrees = 0.01;
    settings.allowMirroring = true;
    settings.performanceProfile = PerformanceProfile::Maximum;
    settings.qualityMode = QualityMode::MaxQuality;
    settings.timeLimitSeconds = 6.0;
    settings.cpuThreadCount = 1;
    settings.deterministic = true;
    settings.randomSeed = 313u;
    settings.collisionTolerance = 0.01;
    settings.placementStrategy = PlacementStrategy::CenterOut;
    return settings;
}

size_t nonZeroGroups(const ActiveMoveSummary& summary) {
    size_t count = 0;
    const size_t values[] = {
        summary.contact, summary.compression, summary.gap, summary.hole,
        summary.concavity, summary.smallPart, summary.swap, summary.chain,
        summary.cluster, summary.region, summary.rotation, summary.mirror,
        summary.escape, summary.frontier
    };
    for (size_t value : values) {
        if (value > 0) {
            ++count;
        }
    }
    return count;
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

bool runPartNeedScheduling() {
    Document document = makeAdaptiveDocument();
    EngineSettings settings = adaptiveSettings();
    std::atomic_bool stopRequested{false};
    MultiStartSolver solver;
    std::vector<ActiveMoveSummary> events;
    LayoutState solved = solver.solve(document, settings, stopRequested, [&](const SolverProgress& progress) {
        if (progress.layoutChanged) {
            events.push_back(progress.activeMoves);
        }
    });
    PenaltySystem penalties;
    solved = LayoutScore{}.evaluate(document, settings, solved.poses, &penalties);
    if (!solved.valid()) {
        std::cout << "partNeed valid=0 collision=" << solved.collisionCount << " invalid=" << solved.invalidPartCount << "\n";
        return false;
    }

    const SolverStats stats = solver.lastStats();
    const bool mixedOperatorsInStep = nonZeroGroups(stats.activeMoveSummary) >= 4;
    const bool earlyHoleOrGap = stats.activeMoveSummary.hole > 0 ||
        stats.activeMoveSummary.concavity > 0 ||
        stats.activeMoveSummary.smallPart > 0 ||
        stats.activeMoveSummary.gap > 0;
    const bool globalNotLocked = nonZeroGroups(stats.activeMoveSummary) >= 4;
    std::cout << "partNeed valid=1 events=" << events.size()
              << " mixedStep=" << mixedOperatorsInStep
              << " earlyHoleOrGap=" << earlyHoleOrGap
              << " activeGroups=" << nonZeroGroups(stats.activeMoveSummary)
              << " bestUpdates=" << stats.bestUpdates
              << " util=" << solved.utilization
              << "\n";
    return mixedOperatorsInStep && earlyHoleOrGap && globalNotLocked;
}

bool runSmallPartHoleUse() {
    Document document;
    document.sheet.width = 190.0;
    document.sheet.height = 145.0;
    document.sheet.margin = 6.0;
    document.addPart(donutPart());
    document.addPart(boxPart(18.0, 18.0));

    EngineSettings settings = adaptiveSettings();
    settings.sheetWidth = document.sheet.width;
    settings.sheetHeight = document.sheet.height;
    settings.timeLimitSeconds = 5.0;
    settings.allowRotation = false;
    settings.rotationMode = RotationMode::None;

    std::atomic_bool stopRequested{false};
    MultiStartSolver solver;
    bool earlyHoleTask = false;
    LayoutState solved = solver.solve(document, settings, stopRequested, [&](const SolverProgress& progress) {
        if (progress.versionId <= 6 && (progress.activeMoves.hole > 0 || progress.activeMoves.smallPart > 0)) {
            earlyHoleTask = true;
        }
    });
    PenaltySystem penalties;
    solved = LayoutScore{}.evaluate(document, settings, solved.poses, &penalties);
    const bool inside = solved.valid() && pointInsideTransformedHole(document, solved, 0, partCenter(document, solved, 1));
    std::cout << "smallHole valid=" << solved.valid()
              << " inside=" << inside
              << " earlyHoleTask=" << earlyHoleTask
              << " bestUpdates=" << solver.lastStats().bestUpdates
              << "\n";
    return inside && earlyHoleTask;
}

bool runSnapshotVersioning() {
    Document document;
    document.sheet.width = 190.0;
    document.sheet.height = 145.0;
    document.sheet.margin = 6.0;
    document.addPart(donutPart());
    document.addPart(boxPart(18.0, 18.0));
    EngineSettings settings = adaptiveSettings();
    settings.sheetWidth = document.sheet.width;
    settings.sheetHeight = document.sheet.height;
    settings.allowRotation = false;
    settings.rotationMode = RotationMode::None;
    settings.timeLimitSeconds = 5.0;

    NestingEngine engine;
    engine.setDocument(&document);
    engine.setSettings(settings);
    engine.start();

    uint64_t lastVersion = 0;
    bool sawAcceptedVersion = false;
    size_t versionChanges = 0;
    const auto started = Clock::now();
    while (engine.isRunning()) {
        const SolverSnapshot snapshot = engine.getLatestSnapshot();
        if (snapshot.versionId != 0 && snapshot.versionId != lastVersion) {
            if (lastVersion != 0) {
                ++versionChanges;
            }
            lastVersion = snapshot.versionId;
            if (snapshot.layoutChanged && snapshot.versionId > 1) {
                sawAcceptedVersion = true;
            }
        }
        if (std::chrono::duration<double>(Clock::now() - started).count() > 15.0) {
            engine.stop();
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
    const SolverSnapshot finalA = engine.getLatestSnapshot();
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    const SolverSnapshot finalB = engine.getLatestSnapshot();
    const SolverResult result = engine.getBestResult();
    std::cout << "snapshot version=" << finalA.versionId
              << " changes=" << versionChanges
              << " acceptedVersion=" << sawAcceptedVersion
              << " stableFinal=" << (finalA.versionId == finalB.versionId)
              << " phase=" << static_cast<int>(finalA.phase)
              << " valid=" << result.valid
              << "\n";
    return sawAcceptedVersion &&
        finalA.versionId == finalB.versionId &&
        finalA.phase == SolverPhase::Done &&
        result.valid;
}

bool runMixed500(const std::filesystem::path& root) {
    const std::filesystem::path input = root / L"samples" / L"benchmark" / L"mixed_500_parts.svg";
    Importer importer;
    ImportResult imported = importer.importFile(input.wstring(), 0.35);
    if (!imported.ok) {
        std::wcerr << L"FAIL: import failed " << input << L"\n";
        return false;
    }

    EngineSettings settings;
    settings.sheetWidth = 3300.0;
    settings.sheetHeight = 2150.0;
    settings.margin = 12.0;
    settings.partSpacing = 0.0;
    settings.allowRotation = false;
    settings.rotationMode = RotationMode::None;
    settings.allowMirroring = true;
    settings.performanceProfile = PerformanceProfile::Maximum;
    settings.qualityMode = QualityMode::MaxQuality;
    settings.timeLimitSeconds = 30.0;
    settings.cpuThreadCount = 1;
    settings.deterministic = true;
    settings.randomSeed = 104u;
    settings.collisionTolerance = 0.01;

    Document document;
    document.sheet.width = settings.sheetWidth;
    document.sheet.height = settings.sheetHeight;
    document.sheet.margin = settings.margin;
    for (Part& part : imported.parts) {
        document.addPart(std::move(part));
    }
    if (document.parts.size() != 500) {
        std::cout << "FAIL: expected 500 parts, got " << document.parts.size() << "\n";
        return false;
    }

    std::atomic_bool stopRequested{false};
    MultiStartSolver solver;
    LayoutState baseline = solver.rowBaseline(document, settings, settings.placementStrategy, settings.randomSeed, 0);
    LayoutState solved = solver.solve(document, settings, stopRequested, {});
    PenaltySystem penalties;
    baseline = LayoutScore{}.evaluate(document, settings, baseline.poses, &penalties);
    solved = LayoutScore{}.evaluate(document, settings, solved.poses, &penalties);
    const SolverStats stats = solver.lastStats();
    std::cout << "mixed500 valid=" << solved.valid()
              << " collision=" << solved.collisionCount
              << " invalid=" << solved.invalidPartCount
              << " spacingPenalty=" << solved.spacingPenalty
              << " sheetPenalty=" << solved.sheetPenalty
              << " baselineUtil=" << baseline.utilization
              << " util=" << solved.utilization
              << " bestUpdates=" << stats.bestUpdates
              << " activeGroups=" << nonZeroGroups(stats.activeMoveSummary)
              << "\n";
    return solved.valid() &&
        solved.collisionCount == 0 &&
        solved.invalidPartCount == 0 &&
        solved.spacingPenalty <= 0.0 &&
        solved.sheetPenalty <= 0.0 &&
        stats.bestUpdates > 0 &&
        solved.utilization + 1e-6 >= baseline.utilization &&
        nonZeroGroups(stats.activeMoveSummary) >= 2;
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    if (argc < 2) {
        std::wcerr << L"usage: adaptive_part_need_smoke <repo-root>\n";
        return 2;
    }
    const std::filesystem::path root = argv[1];
    if (!runPartNeedScheduling()) {
        return fail("part need scheduler did not select mixed per-part operators");
    }
    if (!runSmallPartHoleUse()) {
        return fail("small part was not moved into a hole early");
    }
    if (!runSnapshotVersioning()) {
        return fail("snapshot versioning is not event-driven");
    }
    if (!runMixed500(root)) {
        return fail("mixed_500 adaptive quality did not hold");
    }
    std::cout << "PASS: adaptive part need scheduling and event-driven snapshots\n";
    return 0;
}
