#include "core/document.h"
#include "engine/analytic_contact_candidate.h"
#include "engine/layout_score.h"
#include "engine/multi_start_solver.h"
#include "engine/penalty_system.h"
#include "geometry/collision.h"
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

Part polygonPart(std::initializer_list<Vec2> points, bool hole = false) {
    Part part;
    Ring ring;
    ring.isHole = hole;
    ring.points.assign(points.begin(), points.end());
    part.rings.push_back(std::move(ring));
    part.updateDerivedGeometry();
    return part;
}

Part rectPart(double w, double h) {
    return polygonPart({{0.0, 0.0}, {w, 0.0}, {w, h}, {0.0, h}, {0.0, 0.0}});
}

Part donutPart() {
    Part part = polygonPart({{0.0, 0.0}, {100.0, 0.0}, {100.0, 100.0}, {0.0, 100.0}, {0.0, 0.0}});
    Ring hole;
    hole.isHole = true;
    hole.points = {{30.0, 30.0}, {70.0, 30.0}, {70.0, 70.0}, {30.0, 70.0}, {30.0, 30.0}};
    part.rings.push_back(std::move(hole));
    part.updateDerivedGeometry();
    return part;
}

Part cShapePart() {
    return polygonPart({
        {0.0, 0.0}, {100.0, 0.0}, {100.0, 22.0}, {30.0, 22.0},
        {30.0, 78.0}, {100.0, 78.0}, {100.0, 100.0}, {0.0, 100.0}, {0.0, 0.0}
    });
}

EngineSettings syntheticSettings() {
    EngineSettings settings;
    settings.sheetWidth = 260.0;
    settings.sheetHeight = 220.0;
    settings.margin = 5.0;
    settings.partSpacing = 0.0;
    settings.allowRotation = true;
    settings.rotationMode = RotationMode::RightAngles;
    settings.allowMirroring = true;
    settings.performanceProfile = PerformanceProfile::Maximum;
    settings.qualityMode = QualityMode::MaxQuality;
    settings.collisionTolerance = 0.01;
    return settings;
}

bool hasValidContactCandidate(Document document, const EngineSettings& settings, const char* name) {
    document.sheet.width = settings.sheetWidth;
    document.sheet.height = settings.sheetHeight;
    document.sheet.margin = settings.margin;
    std::vector<Pose> poses(document.parts.size());
    poses[0].x = 45.0;
    poses[0].y = 45.0;
    poses[1].x = 180.0;
    poses[1].y = 120.0;

    AnalyticContactRequest request;
    request.movingPart = 1;
    request.fixedParts = {0};
    request.angles = {0.0, 1.5707963267948966, 3.141592653589793, 4.71238898038469};
    request.mirrors = {false, true};
    request.regionAnchors = {{95.0, 95.0}, {115.0, 95.0}, {95.0, 115.0}};
    request.ownerLimit = 1;
    request.perOwnerPointLimit = 12;
    request.candidateLimit = 256;
    AnalyticContactStats stats;
    const std::vector<AnalyticContactCandidate> candidates =
        AnalyticContactCandidateGenerator{}.generate(document, settings, poses, request, &stats);
    bool valid = false;
    for (const AnalyticContactCandidate& candidate : candidates) {
        if (!partsCollide(document.parts[0], poses[0], document.parts[1], candidate.pose, settings.collisionTolerance)) {
            valid = true;
            break;
        }
    }
    std::cout << name
              << " generated=" << stats.generated
              << " valid=" << stats.valid
              << " collisionReject=" << stats.rejectedCollision
              << " clearanceReject=" << stats.rejectedClearance
              << " sheetReject=" << stats.rejectedSheet
              << "\n";
    return expect(name, !candidates.empty() && valid);
}

EngineSettings benchmarkSettings() {
    EngineSettings settings;
    settings.sheetWidth = 1450.0;
    settings.sheetHeight = 940.0;
    settings.margin = 10.0;
    settings.partSpacing = 2.0;
    settings.performanceProfile = PerformanceProfile::Maximum;
    settings.qualityMode = QualityMode::MaxQuality;
    settings.rotationMode = RotationMode::None;
    settings.allowRotation = false;
    settings.allowMirroring = false;
    settings.timeLimitSeconds = 18.0;
    settings.cpuThreadCount = 1;
    settings.deterministic = true;
    settings.randomSeed = 102u;
    settings.collisionTolerance = 0.01;
    settings.curveFlattenTolerance = 0.35;
    return settings;
}

Document loadMixed100(const std::filesystem::path& root, const EngineSettings& settings) {
    Importer importer;
    const std::filesystem::path input = root / L"samples" / L"benchmark" / L"mixed_100_parts.svg";
    ImportResult imported = importer.importFile(input.wstring(), settings.curveFlattenTolerance);
    Document document;
    document.sheet.width = settings.sheetWidth;
    document.sheet.height = settings.sheetHeight;
    document.sheet.margin = settings.margin;
    for (Part& part : imported.parts) {
        document.addPart(std::move(part));
    }
    return document;
}

bool validLayout(const LayoutState& state) {
    return state.collisionCount == 0 &&
        state.invalidPartCount == 0 &&
        state.spacingPenalty <= 1e-9 &&
        state.sheetPenalty <= 1e-9;
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    if (argc < 2) {
        std::wcerr << L"usage: analytic_contact_candidate_smoke <repo-root>\n";
        return 2;
    }
    const std::filesystem::path root = argv[1];
    const EngineSettings synthetic = syntheticSettings();
    bool ok = true;
    {
        Document document;
        document.addPart(cShapePart());
        document.addPart(rectPart(22.0, 22.0));
        ok = hasValidContactCandidate(std::move(document), synthetic, "c-shape analytic contact") && ok;
    }
    {
        Document document;
        document.addPart(donutPart());
        document.addPart(rectPart(18.0, 18.0));
        ok = hasValidContactCandidate(std::move(document), synthetic, "donut hole analytic contact") && ok;
    }
    {
        Document document;
        document.addPart(polygonPart({{0.0, 0.0}, {80.0, 0.0}, {80.0, 28.0}, {35.0, 28.0}, {35.0, 70.0}, {0.0, 70.0}, {0.0, 0.0}}));
        document.addPart(polygonPart({{0.0, 0.0}, {38.0, 0.0}, {38.0, 38.0}, {16.0, 38.0}, {16.0, 16.0}, {0.0, 16.0}, {0.0, 0.0}}));
        ok = hasValidContactCandidate(std::move(document), synthetic, "puzzle analytic contact") && ok;
    }

    EngineSettings settings = benchmarkSettings();
    Document document = loadMixed100(root, settings);
    std::atomic_bool stopRequested{false};
    MultiStartSolver solver;
    LayoutState solved = solver.solve(document, settings, stopRequested, {});
    PenaltySystem penalties;
    solved = LayoutScore{}.evaluate(document, settings, solved.poses, &penalties);
    const SolverStats stats = solver.lastStats();
    std::cout << "mixed_100"
              << " util=" << solved.utilization
              << " analyticGenerated=" << stats.analyticCandidatesGenerated
              << " analyticValid=" << stats.analyticCandidatesValid
              << " analyticAccepted=" << stats.analyticCandidatesAccepted
              << " contourContactAccepted=" << stats.contourContactAccepted
              << " localRegionRepackAccepted=" << stats.localRegionRepackAccepted
              << " activeMoveAcceptedTotal=" << stats.activeMoveAcceptedTotal
              << " rejectedCollision=" << stats.contactCandidatesRejectedCollision
              << " rejectedClearance=" << stats.contactCandidatesRejectedClearance
              << " rejectedScore=" << stats.contactCandidatesRejectedScore
              << " collision=" << solved.collisionCount
              << " invalid=" << solved.invalidPartCount
              << " spacingPenalty=" << solved.spacingPenalty
              << " sheetPenalty=" << solved.sheetPenalty
              << "\n";

    ok = expect("mixed_100 strict validity", validLayout(solved)) && ok;
    ok = expect("analytic candidates generated", stats.analyticCandidatesGenerated > 0) && ok;
    ok = expect("analytic candidates valid", stats.analyticCandidatesValid > 0) && ok;
    ok = expect("active accepted move total > 20", stats.activeMoveAcceptedTotal > 20) && ok;
    ok = expect("mixed_100 utilization >= 0.65", solved.utilization >= 0.65) && ok;
    return ok ? 0 : 1;
}
