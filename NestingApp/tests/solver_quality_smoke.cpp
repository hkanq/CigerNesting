#include "core/document.h"
#include "engine/compression.h"
#include "engine/layout_score.h"
#include "engine/multi_start_solver.h"
#include "engine/penalty_system.h"
#include "import/importer.h"
#include <algorithm>
#include <atomic>
#include <filesystem>
#include <iostream>
#include <string>
#include <utility>

namespace {

using namespace nest;

Ring boxRing(double x0, double y0, double x1, double y1) {
    Ring ring;
    ring.points = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}, {x0, y0}};
    return ring;
}

Part boxPart(double x0, double y0, double x1, double y1) {
    Part part;
    part.rings.push_back(boxRing(x0, y0, x1, y1));
    part.updateDerivedGeometry();
    return part;
}

Part partFromRings(std::vector<Ring> rings) {
    Part part;
    part.rings = std::move(rings);
    part.updateDerivedGeometry();
    return part;
}

Part donutPart() {
    Ring hole = boxRing(28.0, 28.0, 72.0, 72.0);
    hole.isHole = true;
    return partFromRings({boxRing(0.0, 0.0, 100.0, 100.0), hole});
}

Part bLikePart() {
    Ring holeA = boxRing(22.0, 20.0, 55.0, 48.0);
    Ring holeB = boxRing(22.0, 76.0, 55.0, 105.0);
    holeA.isHole = true;
    holeB.isHole = true;
    return partFromRings({boxRing(0.0, 0.0, 75.0, 130.0), holeA, holeB});
}

Sheet concaveLSheet() {
    Sheet sheet;
    sheet.width = 150.0;
    sheet.height = 150.0;
    sheet.margin = 0.0;
    SheetProfile profile;
    profile.outerContour.points = {
        {0.0, 0.0}, {150.0, 0.0}, {150.0, 55.0}, {55.0, 55.0}, {55.0, 150.0}, {0.0, 150.0}, {0.0, 0.0}
    };
    profile.hasCustomProfile = true;
    sheet.setProfile(profile);
    return sheet;
}

Sheet sheetWithHole() {
    Sheet sheet;
    sheet.width = 180.0;
    sheet.height = 140.0;
    sheet.margin = 0.0;
    SheetProfile profile;
    profile.outerContour = boxRing(0.0, 0.0, 180.0, 140.0);
    Ring hole = boxRing(70.0, 45.0, 110.0, 95.0);
    hole.isHole = true;
    profile.holes.push_back(hole);
    profile.hasCustomProfile = true;
    sheet.setProfile(profile);
    return sheet;
}

Document loadDocument(const std::filesystem::path& root, const wchar_t* sampleName, const EngineSettings& settings) {
    Importer importer;
    const auto imported = importer.importFile((root / L"samples" / sampleName).wstring(), settings.curveFlattenTolerance);
    Document doc;
    doc.sheet.width = settings.sheetWidth;
    doc.sheet.height = settings.sheetHeight;
    doc.sheet.margin = settings.margin;
    for (auto part : imported.parts) {
        doc.addPart(std::move(part));
    }
    return doc;
}

struct QualityCheck {
    bool pass = false;
    bool improved = false;
};

QualityCheck solveAndCompare(const std::wstring& name, const Document& doc, EngineSettings settings) {
    settings.timeLimitSeconds = std::max(0.35, settings.timeLimitSeconds);
    MultiStartSolver solver;
    const LayoutState baseline = solver.rowBaseline(doc, settings, settings.placementStrategy, 1u, 0);
    std::atomic_bool stop{false};
    const LayoutState solved = solver.solve(doc, settings, stop, {});
    const bool valid = solved.valid();
    const bool notWorse = solved.totalScore <= baseline.totalScore + 1e-6 || solved.utilization + 1e-6 >= baseline.utilization;
    const bool improved = solved.valid() && (solved.totalScore + 1e-6 < baseline.totalScore || solved.utilization > baseline.utilization + 1e-6);

    std::wcout << name
               << L" baselineScore=" << baseline.totalScore
               << L" solverScore=" << solved.totalScore
               << L" baselineUtil=" << baseline.utilization
               << L" solverUtil=" << solved.utilization
               << L" collisions=" << solved.collisionCount
               << L" invalid=" << solved.invalidPartCount
               << L" spacingPenalty=" << solved.spacingPenalty << L"\n";
    return {valid && notWorse, improved};
}

QualityCheck samplePasses(const std::filesystem::path& root, const wchar_t* sampleName) {
    EngineSettings settings;
    settings.timeLimitSeconds = 0.75;
    settings.cpuThreadCount = 0;
    settings.placementStrategy = PlacementStrategy::BottomLeft;
    Document doc = loadDocument(root, sampleName, settings);
    if (doc.parts.empty()) {
        std::wcerr << L"FAIL: could not load " << sampleName << L"\n";
        return {};
    }
    return solveAndCompare(sampleName, doc, settings);
}

QualityCheck syntheticDonutPasses() {
    EngineSettings settings;
    settings.sheetWidth = 240.0;
    settings.sheetHeight = 180.0;
    settings.margin = 8.0;
    settings.partSpacing = 4.0;
    settings.timeLimitSeconds = 0.5;
    Document doc;
    doc.sheet.width = settings.sheetWidth;
    doc.sheet.height = settings.sheetHeight;
    doc.sheet.margin = settings.margin;
    doc.addPart(donutPart());
    doc.addPart(boxPart(0.0, 0.0, 20.0, 20.0));
    doc.addPart(boxPart(0.0, 0.0, 18.0, 18.0));
    return solveAndCompare(L"synthetic_donut", doc, settings);
}

QualityCheck syntheticBLikePasses() {
    EngineSettings settings;
    settings.sheetWidth = 260.0;
    settings.sheetHeight = 190.0;
    settings.margin = 8.0;
    settings.partSpacing = 4.0;
    settings.timeLimitSeconds = 0.5;
    settings.allowMirroring = true;
    Document doc;
    doc.sheet.width = settings.sheetWidth;
    doc.sheet.height = settings.sheetHeight;
    doc.sheet.margin = settings.margin;
    doc.addPart(bLikePart());
    doc.addPart(boxPart(0.0, 0.0, 18.0, 18.0));
    doc.addPart(boxPart(0.0, 0.0, 16.0, 16.0));
    return solveAndCompare(L"synthetic_b_like_mirror", doc, settings);
}

QualityCheck syntheticConcaveSheetPasses() {
    EngineSettings settings;
    settings.sheetWidth = 150.0;
    settings.sheetHeight = 150.0;
    settings.margin = 0.0;
    settings.partSpacing = 3.0;
    settings.timeLimitSeconds = 0.5;
    settings.placementStrategy = PlacementStrategy::TopToBottom;
    Document doc;
    doc.sheet = concaveLSheet();
    doc.addPart(boxPart(0.0, 0.0, 36.0, 36.0));
    doc.addPart(boxPart(0.0, 0.0, 34.0, 28.0));
    doc.addPart(boxPart(0.0, 0.0, 26.0, 26.0));
    return solveAndCompare(L"synthetic_concave_sheet", doc, settings);
}

QualityCheck syntheticSheetHolePasses() {
    EngineSettings settings;
    settings.sheetWidth = 180.0;
    settings.sheetHeight = 140.0;
    settings.margin = 0.0;
    settings.partSpacing = 3.0;
    settings.timeLimitSeconds = 0.5;
    settings.placementStrategy = PlacementStrategy::CenterOut;
    Document doc;
    doc.sheet = sheetWithHole();
    doc.addPart(boxPart(0.0, 0.0, 32.0, 32.0));
    doc.addPart(boxPart(0.0, 0.0, 30.0, 24.0));
    doc.addPart(boxPart(0.0, 0.0, 26.0, 26.0));
    return solveAndCompare(L"synthetic_sheet_hole", doc, settings);
}

bool compressionImprovesSynthetic() {
    EngineSettings settings;
    settings.sheetWidth = 220.0;
    settings.sheetHeight = 120.0;
    settings.margin = 10.0;
    settings.partSpacing = 5.0;
    settings.placementStrategy = PlacementStrategy::BottomLeft;

    Document doc;
    doc.sheet.width = settings.sheetWidth;
    doc.sheet.height = settings.sheetHeight;
    doc.sheet.margin = settings.margin;
    doc.addPart(boxPart(0.0, 0.0, 20.0, 20.0));
    doc.addPart(boxPart(0.0, 0.0, 20.0, 20.0));

    LayoutState state;
    state.poses.resize(2);
    state.poses[0].x = 10.0;
    state.poses[0].y = 10.0;
    state.poses[1].x = 170.0;
    state.poses[1].y = 10.0;

    PenaltySystem penalties;
    LayoutScore scorer;
    state = scorer.evaluate(doc, settings, state.poses, &penalties);
    Compression compression;
    const LayoutState compressed = compression.compressByScore(doc, settings, state, penalties);

    std::cout << "compression usedWidth before=" << state.usedWidth
              << " after=" << compressed.usedWidth
              << " valid=" << compressed.valid() << "\n";
    return compressed.valid() && compressed.usedWidth + 1e-6 < state.usedWidth;
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    if (argc < 2) {
        std::wcerr << L"usage: solver_quality_smoke <repo-root>\n";
        return 2;
    }

    const std::filesystem::path root = argv[1];
    bool ok = true;
    int improvedCount = 0;
    const QualityCheck simple = samplePasses(root, L"simple_polygons.svg");
    const QualityCheck shapes = samplePasses(root, L"basic_shapes.svg");
    const QualityCheck donut = syntheticDonutPasses();
    const QualityCheck bLike = syntheticBLikePasses();
    const QualityCheck concave = syntheticConcaveSheetPasses();
    const QualityCheck holedSheet = syntheticSheetHolePasses();
    const QualityCheck checks[] = {simple, shapes, donut, bLike, concave, holedSheet};
    for (const QualityCheck& check : checks) {
        ok = check.pass && ok;
        improvedCount += check.improved ? 1 : 0;
    }
    ok = compressionImprovesSynthetic() && ok;
    std::cout << "improved_scenarios=" << improvedCount << "\n";
    ok = improvedCount >= 2 && ok;
    return ok ? 0 : 1;
}
