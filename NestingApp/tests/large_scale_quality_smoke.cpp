#include "core/document.h"
#include "engine/engine_settings.h"
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

struct CaseSettings {
    const wchar_t* fileName = L"";
    double sheetWidth = 1000.0;
    double sheetHeight = 700.0;
    double margin = 8.0;
    double spacing = 2.0;
    bool allowRotation = false;
    bool allowMirroring = false;
    RotationMode rotationMode = RotationMode::None;
    double rotationStepDegrees = 90.0;
    uint32_t seed = 1u;
};

Document loadDocument(const std::filesystem::path& root, const CaseSettings& caseSettings, const EngineSettings& settings) {
    Importer importer;
    const ImportResult imported = importer.importFile((root / L"samples" / L"benchmark" / caseSettings.fileName).wstring(), settings.curveFlattenTolerance);
    Document document;
    document.sheet.width = settings.sheetWidth;
    document.sheet.height = settings.sheetHeight;
    document.sheet.margin = settings.margin;
    if (!imported.ok) {
        return document;
    }
    for (Part part : imported.parts) {
        document.addPart(std::move(part));
    }
    return document;
}

EngineSettings makeSettings(const CaseSettings& caseSettings, PerformanceProfile profile, double timeLimitSeconds) {
    EngineSettings settings;
    settings.sheetWidth = caseSettings.sheetWidth;
    settings.sheetHeight = caseSettings.sheetHeight;
    settings.margin = caseSettings.margin;
    settings.partSpacing = caseSettings.spacing;
    settings.allowRotation = caseSettings.allowRotation;
    settings.allowMirroring = caseSettings.allowMirroring;
    settings.rotationMode = caseSettings.rotationMode;
    settings.rotationStepDegrees = caseSettings.rotationStepDegrees;
    settings.performanceProfile = profile;
    settings.qualityMode = profile == PerformanceProfile::Maximum ? QualityMode::MaxQuality :
        profile == PerformanceProfile::Fast ? QualityMode::Fast : QualityMode::Balanced;
    settings.timeLimitSeconds = timeLimitSeconds;
    settings.cpuThreadCount = 1;
    settings.randomSeed = caseSettings.seed;
    settings.deterministic = true;
    return settings;
}

struct SolveResult {
    LayoutState state;
    SolverStats stats;
};

SolveResult solve(const Document& document, const EngineSettings& settings) {
    std::atomic_bool stop{false};
    MultiStartSolver solver;
    LayoutState state = solver.solve(document, settings, stop, {});
    LayoutScore scorer;
    PenaltySystem penalties;
    state = scorer.evaluate(document, settings, state.poses, &penalties);
    return {std::move(state), solver.lastStats()};
}

bool expect(const char* name, bool condition) {
    std::cout << (condition ? "PASS: " : "FAIL: ") << name << "\n";
    return condition;
}

bool validLayout(const SolveResult& result) {
    return result.state.collisionCount == 0 &&
        result.state.invalidPartCount == 0 &&
        result.state.spacingPenalty <= 1e-9 &&
        result.state.sheetPenalty <= 1e-9;
}

bool manySmallImproves(const std::filesystem::path& root) {
    const CaseSettings caseSettings{L"many_small_parts.svg", 1150.0, 760.0, 8.0, 2.0, false, false, RotationMode::None, 90.0, 101u};
    const EngineSettings fastSettings = makeSettings(caseSettings, PerformanceProfile::Fast, 0.35);
    const EngineSettings maxSettings = makeSettings(caseSettings, PerformanceProfile::Maximum, 0.85);
    const Document fastDoc = loadDocument(root, caseSettings, fastSettings);
    const Document maxDoc = loadDocument(root, caseSettings, maxSettings);
    const SolveResult fast = solve(fastDoc, fastSettings);
    const SolveResult maximum = solve(maxDoc, maxSettings);
    const double fastArea = fast.state.usedWidth * fast.state.usedHeight;
    const double maximumArea = maximum.state.usedWidth * maximum.state.usedHeight;
    std::cout << "many_small fastUtil=" << fast.state.utilization
              << " maxUtil=" << maximum.state.utilization
              << " maxBestUpdates=" << maximum.stats.bestUpdates << "\n";
    return validLayout(fast) && validLayout(maximum) &&
        maximum.stats.bestUpdates > 1 &&
        (maximum.state.utilization > fast.state.utilization + 1e-6 || maximumArea + 1e-6 < fastArea);
}

bool mixed100Updates(const std::filesystem::path& root) {
    const CaseSettings caseSettings{L"mixed_100_parts.svg", 1450.0, 940.0, 10.0, 2.0, false, false, RotationMode::None, 90.0, 102u};
    const EngineSettings settings = makeSettings(caseSettings, PerformanceProfile::Maximum, 0.55);
    const Document document = loadDocument(root, caseSettings, settings);
    const SolveResult result = solve(document, settings);
    std::cout << "mixed100 util=" << result.state.utilization
              << " bestUpdates=" << result.stats.bestUpdates << "\n";
    return validLayout(result) && result.stats.bestUpdates > 0;
}

bool mixed500Updates(const std::filesystem::path& root) {
    const CaseSettings caseSettings{L"mixed_500_parts.svg", 3300.0, 2150.0, 12.0, 2.0, false, false, RotationMode::None, 90.0, 104u};
    const EngineSettings settings = makeSettings(caseSettings, PerformanceProfile::Maximum, 3.0);
    const Document document = loadDocument(root, caseSettings, settings);
    const SolveResult result = solve(document, settings);
    std::cout << "mixed500 util=" << result.state.utilization
              << " bestUpdates=" << result.stats.bestUpdates
              << " compactionAccepted=" << result.stats.compactionAccepted << "\n";
    return validLayout(result) && result.stats.bestUpdates > 0;
}

bool mirrorEnabledBeatsDisabled(const std::filesystem::path& root) {
    CaseSettings disabled{L"mirror_required.svg", 760.0, 420.0, 8.0, 2.0, true, false, RotationMode::FortyFiveDegrees, 45.0, 109u};
    CaseSettings enabled = disabled;
    enabled.allowMirroring = true;
    const EngineSettings disabledSettings = makeSettings(disabled, PerformanceProfile::Balanced, 1.0);
    const EngineSettings enabledSettings = makeSettings(enabled, PerformanceProfile::Balanced, 1.0);
    const SolveResult noMirror = solve(loadDocument(root, disabled, disabledSettings), disabledSettings);
    const SolveResult withMirror = solve(loadDocument(root, enabled, enabledSettings), enabledSettings);
    std::cout << "mirror disabledScore=" << noMirror.state.totalScore
              << " enabledScore=" << withMirror.state.totalScore << "\n";
    return validLayout(noMirror) && validLayout(withMirror) &&
        withMirror.state.totalScore <= noMirror.state.totalScore + 1e-6;
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    if (argc < 2) {
        std::wcerr << L"usage: large_scale_quality_smoke <repo-root>\n";
        return 2;
    }
    const std::filesystem::path root = argv[1];
    bool ok = true;
    ok = expect("many small parts compaction improves used area", manySmallImproves(root)) && ok;
    ok = expect("mixed_100 produces at least one best update", mixed100Updates(root)) && ok;
    ok = expect("mixed_500 produces at least one Maximum improvement", mixed500Updates(root)) && ok;
    ok = expect("mirror enabled is better or equal", mirrorEnabledBeatsDisabled(root)) && ok;
    return ok ? 0 : 1;
}
