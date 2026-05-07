#include "core/document.h"
#include "engine/empty_space_map.h"
#include "engine/layout_score.h"
#include "engine/multi_start_solver.h"
#include "engine/penalty_system.h"
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

EngineSettings settings() {
    EngineSettings s;
    s.sheetWidth = 1450.0;
    s.sheetHeight = 940.0;
    s.margin = 10.0;
    s.partSpacing = 2.0;
    s.performanceProfile = PerformanceProfile::Maximum;
    s.qualityMode = QualityMode::MaxQuality;
    s.rotationMode = RotationMode::None;
    s.allowRotation = false;
    s.allowMirroring = false;
    s.timeLimitSeconds = 0.0;
    s.cpuThreadCount = 1;
    s.deterministic = true;
    s.randomSeed = 102u;
    s.collisionTolerance = 0.01;
    s.curveFlattenTolerance = 0.35;
    return s;
}

Document loadMixed100(const std::filesystem::path& root, const EngineSettings& s) {
    Importer importer;
    ImportResult imported = importer.importFile((root / L"samples" / L"benchmark" / L"mixed_100_parts.svg").wstring(), s.curveFlattenTolerance);
    Document document;
    document.sheet.width = s.sheetWidth;
    document.sheet.height = s.sheetHeight;
    document.sheet.margin = s.margin;
    for (Part& part : imported.parts) {
        document.addPart(std::move(part));
    }
    return document;
}

bool valid(const LayoutState& state) {
    return state.collisionCount == 0 && state.invalidPartCount == 0 && state.spacingPenalty <= 1e-9 && state.sheetPenalty <= 1e-9;
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    if (argc < 2) {
        std::wcerr << L"usage: constructive_rebuild_smoke <repo-root>\n";
        return 2;
    }
    EngineSettings s = settings();
    Document document = loadMixed100(argv[1], s);
    std::atomic_bool stop{false};
    MultiStartSolver solver;
    LayoutState solved = solver.solve(document, s, stop, {});
    PenaltySystem penalties;
    solved = LayoutScore{}.evaluate(document, s, solved.poses, &penalties);
    EmptySpaceMap map = EmptySpaceAnalyzer{}.analyze(document, s, solved);
    SolverStats stats = solver.lastStats();
    std::cout << "constructive mixed_100"
              << " util=" << solved.utilization
              << " destroyAttempts=" << stats.destroyAttempts
              << " destroyAccepted=" << stats.destroyAccepted
              << " avgSubset=" << stats.averageSubsetSize
              << " beamNodes=" << stats.beamNodesExpanded
              << " beamLeaves=" << stats.beamValidLeaves
              << " largestGap=" << map.largestRegionArea
              << " activeAccepted=" << stats.activeMoveAcceptedTotal
              << "\n";
    bool ok = true;
    ok = expect("strict validity", valid(solved)) && ok;
    ok = expect("constructive rebuild is main search", stats.destroyAttempts >= 20) && ok;
    ok = expect("multi-part beam search expanded", stats.beamNodesExpanded >= 1000) && ok;
    ok = expect("active moves are not zero", stats.activeMoveAcceptedTotal > 20) && ok;
    ok = expect("mixed_100 does not regress below contour baseline", solved.utilization >= 0.52) && ok;
    return ok ? 0 : 1;
}
