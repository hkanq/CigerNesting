#include "core/document.h"
#include "engine/compression.h"
#include "engine/layout_score.h"
#include "engine/multi_start_solver.h"
#include "engine/penalty_system.h"
#include "import/importer.h"
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

bool samplePasses(const std::filesystem::path& root, const wchar_t* sampleName) {
    EngineSettings settings;
    settings.timeLimitSeconds = 0.75;
    settings.cpuThreadCount = 0;
    settings.placementStrategy = PlacementStrategy::BottomLeft;
    Document doc = loadDocument(root, sampleName, settings);
    if (doc.parts.empty()) {
        std::wcerr << L"FAIL: could not load " << sampleName << L"\n";
        return false;
    }

    MultiStartSolver solver;
    const LayoutState baseline = solver.rowBaseline(doc, settings, PlacementStrategy::BottomLeft, 1u, 0);
    std::atomic_bool stop{false};
    const LayoutState solved = solver.solve(doc, settings, stop, {});

    const bool valid = solved.valid();
    const bool notWorse = solved.totalScore <= baseline.totalScore + 1e-6 || solved.utilization + 1e-6 >= baseline.utilization;
    std::wcout << sampleName
               << L" baselineScore=" << baseline.totalScore
               << L" solverScore=" << solved.totalScore
               << L" baselineUtil=" << baseline.utilization
               << L" solverUtil=" << solved.utilization
               << L" collisions=" << solved.collisionCount
               << L" invalid=" << solved.invalidPartCount << L"\n";
    return valid && notWorse;
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
    ok = samplePasses(root, L"simple_polygons.svg") && ok;
    ok = samplePasses(root, L"basic_shapes.svg") && ok;
    ok = compressionImprovesSynthetic() && ok;
    return ok ? 0 : 1;
}
