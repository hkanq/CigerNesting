#include "core/document.h"
#include "engine/empty_space_map.h"
#include "engine/layout_score.h"
#include "engine/layout_score_components.h"
#include "engine/multi_start_solver.h"
#include "engine/penalty_system.h"
#include "geometry/transformed_shape.h"
#include "import/importer.h"

#include <atomic>
#include <filesystem>
#include <iostream>
#include <utility>

namespace {

using namespace nest;

bool expect(const char* name, bool condition) {
    std::cout << (condition ? "PASS: " : "QUALITY_FAIL: ") << name << "\n";
    return condition;
}

EngineSettings makeSettings(double width, double height, double margin, double spacing, uint32_t seed, double seconds) {
    EngineSettings settings;
    settings.sheetWidth = width;
    settings.sheetHeight = height;
    settings.margin = margin;
    settings.partSpacing = spacing;
    settings.performanceProfile = PerformanceProfile::Maximum;
    settings.qualityMode = QualityMode::MaxQuality;
    settings.rotationMode = RotationMode::None;
    settings.allowRotation = false;
    settings.allowMirroring = false;
    settings.timeLimitSeconds = seconds;
    settings.cpuThreadCount = 1;
    settings.deterministic = true;
    settings.randomSeed = seed;
    settings.collisionTolerance = 0.01;
    settings.curveFlattenTolerance = 0.35;
    return settings;
}

Document load(const std::filesystem::path& root, const wchar_t* fileName, const EngineSettings& settings) {
    Importer importer;
    ImportResult imported = importer.importFile((root / L"samples" / L"benchmark" / fileName).wstring(), settings.curveFlattenTolerance);
    Document document;
    document.sheet.width = settings.sheetWidth;
    document.sheet.height = settings.sheetHeight;
    document.sheet.margin = settings.margin;
    if (!imported.ok) {
        return document;
    }
    for (Part& part : imported.parts) {
        document.addPart(std::move(part));
    }
    return document;
}

bool valid(const LayoutState& state) {
    return state.collisionCount == 0 &&
        state.invalidPartCount == 0 &&
        state.spacingPenalty <= 1e-9 &&
        state.sheetPenalty <= 1e-9;
}

LayoutShapeMetrics shapeMetrics(const Document& document, const EngineSettings& settings, const LayoutState& state) {
    AABB used;
    for (size_t i = 0; i < document.parts.size() && i < state.poses.size(); ++i) {
        used.include(transformedBounds(document.parts[i], state.poses[i]));
    }
    return computeLayoutShapeMetrics(document, settings, used);
}

bool runCase(const std::filesystem::path& root, const wchar_t* fileName, const char* name, EngineSettings settings, double targetUtilization) {
    Document document = load(root, fileName, settings);
    std::atomic_bool stop{false};
    MultiStartSolver solver;
    LayoutState solved = solver.solve(document, settings, stop, {});
    PenaltySystem penalties;
    solved = LayoutScore{}.evaluate(document, settings, solved.poses, &penalties);
    const SolverStats stats = solver.lastStats();
    const LayoutShapeMetrics metrics = shapeMetrics(document, settings, solved);
    std::cout << name
              << " util=" << solved.utilization
              << " tower=" << metrics.towerScore
              << " localRegionRepackAccepted=" << stats.localRegionRepackAccepted
              << " localRegionRepackSubsets=" << stats.localRegionRepackSubsets
              << " collision=" << solved.collisionCount
              << " invalid=" << solved.invalidPartCount
              << " spacingPenalty=" << solved.spacingPenalty
              << " sheetPenalty=" << solved.sheetPenalty
              << "\n";
    bool ok = true;
    ok = expect("validity remains strict", valid(solved)) && ok;
    ok = expect("local region repack was attempted", stats.localRegionRepackSubsets > 0) && ok;
    ok = expect("utilization target", solved.utilization + 1e-9 >= targetUtilization) && ok;
    return ok;
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    if (argc < 2) {
        std::wcerr << L"usage: local_region_repack_smoke <repo-root>\n";
        return 2;
    }
    const std::filesystem::path root = argv[1];
    bool ok = true;
    ok = runCase(root, L"mixed_100_parts.svg", "mixed_100_parts", makeSettings(1450.0, 940.0, 10.0, 2.0, 102u, 18.0), 0.72) && ok;
    ok = runCase(root, L"mixed_500_parts.svg", "mixed_500_parts", makeSettings(3300.0, 2150.0, 12.0, 2.0, 104u, 30.0), 0.65) && ok;
    return ok ? 0 : 1;
}
