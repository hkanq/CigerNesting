#include "core/document.h"
#include "engine/engine_settings.h"
#include "engine/layout_score.h"
#include "engine/multi_start_solver.h"
#include "engine/penalty_system.h"
#include "geometry/transformed_shape.h"
#include "import/importer.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace nest;
using Clock = std::chrono::steady_clock;

struct BenchmarkCase {
    std::string name;
    std::wstring fileName;
    double sheetWidth = 1000.0;
    double sheetHeight = 700.0;
    double margin = 8.0;
    double spacing = 2.0;
    bool customSheet = false;
    bool allowRotation = true;
    bool allowMirroring = false;
    RotationMode rotationMode = RotationMode::RightAngles;
    double rotationStepDegrees = 90.0;
    PlacementStrategy placementStrategy = PlacementStrategy::BottomLeft;
    uint32_t seed = 1u;
};

struct BenchmarkRow {
    std::string caseName;
    size_t partCount = 0;
    PerformanceProfile profile = PerformanceProfile::Fast;
    double elapsedMs = 0.0;
    double utilization = 0.0;
    double usedWidth = 0.0;
    double usedHeight = 0.0;
    int collisions = 0;
    int invalid = 0;
    double spacingPenalty = 0.0;
    double sheetPenalty = 0.0;
    size_t bestUpdates = 0;
    size_t evaluatedCandidates = 0;
    double candidatesPerSecond = 0.0;
    size_t acceptedMoves = 0;
    size_t acceptedWorseMoves = 0;
    size_t gapAccepted = 0;
    size_t swapAccepted = 0;
    size_t chainAccepted = 0;
    size_t escapeAccepted = 0;
    size_t ultraAccepted = 0;
    size_t compactionAccepted = 0;
    size_t frontierCandidates = 0;
    size_t smallFillerAccepted = 0;
    size_t regionRepackAccepted = 0;
    double finalScore = 0.0;
    bool qualityPass = false;
    bool profilePass = false;
    std::string failureReason;
};

const char* profileName(PerformanceProfile profile) {
    switch (profile) {
    case PerformanceProfile::Fast:
        return "Fast";
    case PerformanceProfile::Balanced:
        return "Balanced";
    case PerformanceProfile::Maximum:
        return "Maximum";
    default:
        return "Unknown";
    }
}

std::string profileFileName(PerformanceProfile profile) {
    switch (profile) {
    case PerformanceProfile::Fast:
        return "fast";
    case PerformanceProfile::Balanced:
        return "balanced";
    case PerformanceProfile::Maximum:
        return "maximum";
    default:
        return "unknown";
    }
}

const char* rotationModeName(RotationMode mode) {
    switch (mode) {
    case RotationMode::None:
        return "None";
    case RotationMode::RightAngles:
        return "RightAngles";
    case RotationMode::FortyFiveDegrees:
        return "FortyFiveDegrees";
    case RotationMode::FixedStep:
        return "FixedStep";
    case RotationMode::ContinuousRefine:
        return "ContinuousRefine";
    default:
        return "Unknown";
    }
}

const char* placementStrategyName(PlacementStrategy strategy) {
    switch (strategy) {
    case PlacementStrategy::BottomLeft:
        return "BottomLeft";
    case PlacementStrategy::TopLeft:
        return "TopLeft";
    case PlacementStrategy::BottomRight:
        return "BottomRight";
    case PlacementStrategy::TopRight:
        return "TopRight";
    case PlacementStrategy::LeftToRight:
        return "LeftToRight";
    case PlacementStrategy::RightToLeft:
        return "RightToLeft";
    case PlacementStrategy::TopToBottom:
        return "TopToBottom";
    case PlacementStrategy::BottomToTop:
        return "BottomToTop";
    case PlacementStrategy::CenterOut:
        return "CenterOut";
    case PlacementStrategy::OutsideIn:
        return "OutsideIn";
    case PlacementStrategy::UserPoints:
        return "UserPoints";
    default:
        return "Unknown";
    }
}

std::string jsonEscape(const std::string& value) {
    std::string out;
    out.reserve(value.size() + 8);
    for (char ch : value) {
        switch (ch) {
        case '\\':
            out += "\\\\";
            break;
        case '"':
            out += "\\\"";
            break;
        case '\n':
            out += "\\n";
            break;
        case '\r':
            out += "\\r";
            break;
        case '\t':
            out += "\\t";
            break;
        default:
            out += ch;
            break;
        }
    }
    return out;
}

std::string narrowPath(const std::filesystem::path& path) {
    return path.generic_string();
}

bool scoreNoWorse(double candidate, double reference) {
    const double tolerance = std::max(1.0, std::abs(reference)) * 1e-6;
    return candidate <= reference + tolerance;
}

bool poseAlmostEqual(const Pose& a, const Pose& b) {
    return std::abs(a.x - b.x) <= 1e-8 &&
        std::abs(a.y - b.y) <= 1e-8 &&
        std::abs(a.angleRadians - b.angleRadians) <= 1e-10 &&
        a.mirrored == b.mirrored;
}

bool layoutAlmostEqual(const LayoutState& a, const LayoutState& b) {
    if (a.poses.size() != b.poses.size()) {
        return false;
    }
    if (std::abs(a.totalScore - b.totalScore) > std::max(1.0, std::abs(a.totalScore)) * 1e-8) {
        return false;
    }
    for (size_t i = 0; i < a.poses.size(); ++i) {
        if (!poseAlmostEqual(a.poses[i], b.poses[i])) {
            return false;
        }
    }
    return true;
}

double timeLimitFor(size_t partCount, PerformanceProfile profile) {
    if (partCount <= 40) {
        switch (profile) {
        case PerformanceProfile::Fast: return 0.18;
        case PerformanceProfile::Balanced: return 1.00;
        case PerformanceProfile::Maximum: return 8.00;
        default: return 1.00;
        }
    }
    if (partCount <= 120) {
        switch (profile) {
        case PerformanceProfile::Fast: return 0.22;
        case PerformanceProfile::Balanced: return 0.35;
        case PerformanceProfile::Maximum: return 0.55;
        default: return 0.35;
        }
    }
    if (partCount <= 275) {
        switch (profile) {
        case PerformanceProfile::Fast: return 0.28;
        case PerformanceProfile::Balanced: return 0.45;
        case PerformanceProfile::Maximum: return 0.70;
        default: return 0.45;
        }
    }
    switch (profile) {
    case PerformanceProfile::Fast: return 0.35;
    case PerformanceProfile::Balanced: return 1.50;
    case PerformanceProfile::Maximum: return 3.00;
    default: return 0.55;
    }
}

QualityMode qualityFor(PerformanceProfile profile) {
    switch (profile) {
    case PerformanceProfile::Fast:
        return QualityMode::Fast;
    case PerformanceProfile::Maximum:
        return QualityMode::MaxQuality;
    case PerformanceProfile::Balanced:
    default:
        return QualityMode::Balanced;
    }
}

EngineSettings settingsFor(const BenchmarkCase& benchmarkCase, size_t partCount, PerformanceProfile profile) {
    EngineSettings settings;
    settings.sheetWidth = benchmarkCase.sheetWidth;
    settings.sheetHeight = benchmarkCase.sheetHeight;
    settings.margin = benchmarkCase.margin;
    settings.partSpacing = benchmarkCase.spacing;
    settings.placementStrategy = benchmarkCase.placementStrategy;
    settings.allowRotation = benchmarkCase.allowRotation;
    settings.rotationMode = benchmarkCase.rotationMode;
    settings.rotationStepDegrees = benchmarkCase.rotationStepDegrees;
    settings.allowMirroring = benchmarkCase.allowMirroring;
    settings.qualityMode = qualityFor(profile);
    settings.performanceProfile = profile;
    settings.timeLimitSeconds = timeLimitFor(partCount, profile);
    settings.cpuThreadCount = 1;
    settings.livePreviewIntervalMs = 250;
    settings.collisionTolerance = 0.01;
    settings.curveFlattenTolerance = 0.35;
    settings.randomSeed = benchmarkCase.seed;
    settings.deterministic = true;
    return settings;
}

Document buildDocument(const BenchmarkCase& benchmarkCase, const std::filesystem::path& inputFile, const EngineSettings& settings, bool& ok, std::wstring& message) {
    Importer importer;
    ImportResult imported = importer.importFile(inputFile.wstring(), settings.curveFlattenTolerance);
    Document document;
    document.sourcePath = inputFile.wstring();
    document.sheet.width = settings.sheetWidth;
    document.sheet.height = settings.sheetHeight;
    document.sheet.margin = settings.margin;
    ok = imported.ok;
    message = imported.message;
    if (!imported.ok) {
        return document;
    }

    size_t firstPart = 0;
    if (benchmarkCase.customSheet) {
        if (imported.parts.size() < 3 || imported.parts[0].rings.empty() || imported.parts[1].rings.empty()) {
            ok = false;
            message = L"Custom sheet benchmark needs sheet outer, sheet hole, and at least one part";
            return document;
        }
        SheetProfile profile;
        profile.outerContour = imported.parts[0].rings.front();
        profile.outerContour.isHole = false;
        Ring hole = imported.parts[1].rings.front();
        hole.isHole = true;
        profile.holes.push_back(std::move(hole));
        profile.hasCustomProfile = true;
        document.sheet.setProfile(profile);
        firstPart = 2;
    }

    for (size_t i = firstPart; i < imported.parts.size(); ++i) {
        document.addPart(std::move(imported.parts[i]));
    }
    if (document.parts.empty()) {
        ok = false;
        message = L"No benchmark parts after sheet extraction";
    }
    return document;
}

void writeRingPath(std::ostream& out, const std::vector<Vec2>& points) {
    if (points.empty()) {
        return;
    }
    out << "M " << points.front().x << ' ' << points.front().y << ' ';
    for (size_t i = 1; i < points.size(); ++i) {
        out << "L " << points[i].x << ' ' << points[i].y << ' ';
    }
    out << 'Z';
}

void writeSvgOutput(
    const std::filesystem::path& outputPath,
    const Document& document,
    const LayoutState& state,
    const BenchmarkCase& benchmarkCase,
    PerformanceProfile profile) {
    std::ofstream out(outputPath);
    if (!out) {
        return;
    }

    out << std::fixed << std::setprecision(3);
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << document.sheet.width
        << "\" height=\"" << document.sheet.height << "\" viewBox=\"0 0 "
        << document.sheet.width << ' ' << document.sheet.height << "\">\n";
    out << "<rect x=\"0\" y=\"0\" width=\"" << document.sheet.width
        << "\" height=\"" << document.sheet.height << "\" fill=\"#f7f7f2\" />\n";
    out << "<g fill=\"none\" stroke=\"#2a2a2a\" stroke-width=\"1.2\">\n";
    if (document.sheet.hasCustomProfile() && !document.sheet.profile().outerContour.points.empty()) {
        out << "<path d=\"";
        writeRingPath(out, document.sheet.profile().outerContour.points);
        out << "\" stroke=\"#1f5f8b\" />\n";
        for (const Ring& hole : document.sheet.profile().holes) {
            out << "<path d=\"";
            writeRingPath(out, hole.points);
            out << "\" stroke=\"#a83232\" stroke-dasharray=\"6 4\" />\n";
        }
        for (const Ring& zone : document.sheet.profile().forbiddenZones) {
            out << "<path d=\"";
            writeRingPath(out, zone.points);
            out << "\" stroke=\"#b88a00\" stroke-dasharray=\"4 3\" />\n";
        }
    } else {
        const Ring rect = document.sheet.makeRectangularOuterContour();
        out << "<path d=\"";
        writeRingPath(out, rect.points);
        out << "\" stroke=\"#1f5f8b\" />\n";
    }
    out << "</g>\n";
    out << "<g fill=\"#86b8e7\" fill-opacity=\"0.58\" stroke=\"#163b5c\" stroke-width=\"0.8\">\n";
    for (size_t i = 0; i < document.parts.size() && i < state.poses.size(); ++i) {
        const TransformedPart transformed = transformPart(document.parts[i], state.poses[i], static_cast<int>(i));
        out << "<g id=\"part-" << i << "\">\n";
        for (const TransformedRing& ring : transformed.rings) {
            out << "<path d=\"";
            writeRingPath(out, ring.points);
            out << "\"";
            if (ring.isHole) {
                out << " fill=\"#f7f7f2\" fill-opacity=\"1\" stroke=\"#163b5c\"";
            }
            out << " />\n";
        }
        out << "</g>\n";
    }
    out << "</g>\n";
    out << "<text x=\"10\" y=\"20\" font-size=\"14\" fill=\"#222\">"
        << benchmarkCase.name << " / " << profileName(profile) << "</text>\n";
    out << "</svg>\n";
}

void writeJsonOutput(
    const std::filesystem::path& outputPath,
    const std::filesystem::path& inputFile,
    const BenchmarkCase& benchmarkCase,
    const EngineSettings& settings,
    const Document& document,
    const LayoutState& state,
    const SolverStats& stats,
    PerformanceProfile profile,
    double elapsedMs) {
    std::ofstream out(outputPath);
    if (!out) {
        return;
    }

    out << std::fixed << std::setprecision(6);
    out << "{\n";
    out << "  \"inputFile\": \"" << jsonEscape(narrowPath(inputFile)) << "\",\n";
    out << "  \"caseName\": \"" << jsonEscape(benchmarkCase.name) << "\",\n";
    out << "  \"profile\": \"" << profileName(profile) << "\",\n";
    out << "  \"settings\": {\n";
    out << "    \"sheetWidth\": " << settings.sheetWidth << ",\n";
    out << "    \"sheetHeight\": " << settings.sheetHeight << ",\n";
    out << "    \"partSpacing\": " << settings.partSpacing << ",\n";
    out << "    \"margin\": " << settings.margin << ",\n";
    out << "    \"rotationMode\": \"" << rotationModeName(settings.rotationMode) << "\",\n";
    out << "    \"rotationStepDegrees\": " << settings.rotationStepDegrees << ",\n";
    out << "    \"allowMirroring\": " << (settings.allowMirroring ? "true" : "false") << ",\n";
    out << "    \"placementStrategy\": \"" << placementStrategyName(settings.placementStrategy) << "\",\n";
    out << "    \"performanceProfile\": \"" << profileName(settings.performanceProfile) << "\",\n";
    out << "    \"timeLimitSeconds\": " << settings.timeLimitSeconds << ",\n";
    out << "    \"cpuThreadCount\": " << settings.cpuThreadCount << ",\n";
    out << "    \"randomSeed\": " << settings.randomSeed << ",\n";
    out << "    \"deterministic\": " << (settings.deterministic ? "true" : "false") << "\n";
    out << "  },\n";
    out << "  \"sheet\": {\n";
    out << "    \"width\": " << document.sheet.width << ",\n";
    out << "    \"height\": " << document.sheet.height << ",\n";
    out << "    \"margin\": " << document.sheet.margin << ",\n";
    out << "    \"hasCustomProfile\": " << (document.sheet.hasCustomProfile() ? "true" : "false") << ",\n";
    out << "    \"holeCount\": " << document.sheet.profile().holes.size() << ",\n";
    out << "    \"forbiddenZoneCount\": " << document.sheet.profile().forbiddenZones.size() << "\n";
    out << "  },\n";
    out << "  \"partsCount\": " << document.parts.size() << ",\n";
    out << "  \"finalPoses\": [\n";
    for (size_t i = 0; i < state.poses.size(); ++i) {
        const Pose& pose = state.poses[i];
        out << "    {\"part\": " << i
            << ", \"x\": " << pose.x
            << ", \"y\": " << pose.y
            << ", \"angleRadians\": " << pose.angleRadians
            << ", \"mirrored\": " << (pose.mirrored ? "true" : "false") << "}";
        out << (i + 1 < state.poses.size() ? "," : "") << "\n";
    }
    out << "  ],\n";
    out << "  \"score\": {\n";
    out << "    \"totalScore\": " << state.totalScore << ",\n";
    out << "    \"utilization\": " << state.utilization << ",\n";
    out << "    \"usedWidth\": " << state.usedWidth << ",\n";
    out << "    \"usedHeight\": " << state.usedHeight << ",\n";
    out << "    \"collisions\": " << state.collisionCount << ",\n";
    out << "    \"invalid\": " << state.invalidPartCount << ",\n";
    out << "    \"spacingPenalty\": " << state.spacingPenalty << ",\n";
    out << "    \"sheetPenalty\": " << state.sheetPenalty << "\n";
    out << "  },\n";
    out << "  \"stats\": {\n";
    out << "    \"elapsedMs\": " << elapsedMs << ",\n";
    out << "    \"evaluatedCandidates\": " << stats.evaluatedCandidates << ",\n";
    out << "    \"candidatesPerSecond\": " << stats.candidatesPerSecond << ",\n";
    out << "    \"acceptedMoves\": " << stats.acceptedMoves << ",\n";
    out << "    \"acceptedWorseMoves\": " << stats.acceptedWorseMoves << ",\n";
    out << "    \"bestUpdates\": " << stats.bestUpdates << ",\n";
    out << "    \"gapAccepted\": " << stats.gapAccepted << ",\n";
    out << "    \"swapAccepted\": " << stats.swapAccepted << ",\n";
    out << "    \"chainAccepted\": " << stats.chainAccepted << ",\n";
    out << "    \"escapeAccepted\": " << stats.escapeAccepted << ",\n";
    out << "    \"ultraAccepted\": " << stats.ultraAccepted << ",\n";
    out << "    \"compactionAccepted\": " << stats.compactionAccepted << ",\n";
    out << "    \"frontierCandidates\": " << stats.frontierCandidates << ",\n";
    out << "    \"smallFillerAccepted\": " << stats.smallFillerAccepted << ",\n";
    out << "    \"regionRepackAccepted\": " << stats.regionRepackAccepted << "\n";
    out << "  }\n";
    out << "}\n";
}

BenchmarkRow runBenchmark(
    const BenchmarkCase& benchmarkCase,
    PerformanceProfile profile,
    const std::filesystem::path& root,
    const std::filesystem::path& outputDir) {
    const std::filesystem::path inputFile = root / L"samples" / L"benchmark" / benchmarkCase.fileName;
    EngineSettings preliminarySettings = settingsFor(benchmarkCase, 1, profile);
    bool importOk = false;
    std::wstring importMessage;
    Document document = buildDocument(benchmarkCase, inputFile, preliminarySettings, importOk, importMessage);

    BenchmarkRow row;
    row.caseName = benchmarkCase.name;
    row.profile = profile;
    row.partCount = document.parts.size();
    if (!importOk) {
        row.failureReason = "import failed";
        std::wcerr << L"FAIL import " << benchmarkCase.fileName << L": " << importMessage << L"\n";
        return row;
    }

    EngineSettings settings = settingsFor(benchmarkCase, document.parts.size(), profile);
    document.sheet.width = settings.sheetWidth;
    document.sheet.height = settings.sheetHeight;
    document.sheet.margin = settings.margin;

    std::atomic_bool stopRequested{false};
    MultiStartSolver solver;
    const auto started = Clock::now();
    LayoutState solved = solver.solve(document, settings, stopRequested, {});
    const double elapsedMs = std::chrono::duration<double, std::milli>(Clock::now() - started).count();
    SolverStats stats = solver.lastStats();
    stats.elapsedMs = elapsedMs;
    stats.candidatesPerSecond = static_cast<double>(stats.evaluatedCandidates) / std::max(0.001, elapsedMs / 1000.0);

    LayoutScore scorer;
    PenaltySystem penalties;
    solved = scorer.evaluate(document, settings, solved.poses, &penalties);

    row.elapsedMs = elapsedMs;
    row.utilization = solved.utilization;
    row.usedWidth = solved.usedWidth;
    row.usedHeight = solved.usedHeight;
    row.collisions = solved.collisionCount;
    row.invalid = solved.invalidPartCount;
    row.spacingPenalty = solved.spacingPenalty;
    row.sheetPenalty = solved.sheetPenalty;
    row.bestUpdates = stats.bestUpdates;
    row.evaluatedCandidates = stats.evaluatedCandidates;
    row.candidatesPerSecond = stats.candidatesPerSecond;
    row.acceptedMoves = stats.acceptedMoves;
    row.acceptedWorseMoves = stats.acceptedWorseMoves;
    row.gapAccepted = stats.gapAccepted;
    row.swapAccepted = stats.swapAccepted;
    row.chainAccepted = stats.chainAccepted;
    row.escapeAccepted = stats.escapeAccepted;
    row.ultraAccepted = stats.ultraAccepted;
    row.compactionAccepted = stats.compactionAccepted;
    row.frontierCandidates = stats.frontierCandidates;
    row.smallFillerAccepted = stats.smallFillerAccepted;
    row.regionRepackAccepted = stats.regionRepackAccepted;
    row.finalScore = solved.totalScore;
    row.qualityPass = solved.collisionCount == 0 &&
        solved.invalidPartCount == 0 &&
        solved.spacingPenalty <= 1e-9 &&
        solved.sheetPenalty <= 1e-9;
    row.profilePass = row.qualityPass;
    if (!row.qualityPass) {
        row.failureReason = "quality rule failed";
    }

    const std::string outputStem = benchmarkCase.name + "_" + profileFileName(profile);
    writeJsonOutput(outputDir / (outputStem + ".cignest.json"), inputFile, benchmarkCase, settings, document, solved, stats, profile, elapsedMs);
    writeSvgOutput(outputDir / (outputStem + ".result.svg"), document, solved, benchmarkCase, profile);
    return row;
}

void writeCsv(const std::filesystem::path& csvPath, const std::vector<BenchmarkRow>& rows) {
    std::ofstream csv(csvPath);
    if (!csv) {
        return;
    }
    csv << "caseName,partCount,profile,elapsedMs,utilization,usedWidth,usedHeight,collisions,invalid,spacingPenalty,sheetPenalty,bestUpdates,evaluatedCandidates,candidatesPerSecond,acceptedMoves,acceptedWorseMoves,gapAccepted,swapAccepted,chainAccepted,escapeAccepted,ultraAccepted,compactionAccepted,frontierCandidates,smallFillerAccepted,regionRepackAccepted,finalScore,status\n";
    csv << std::fixed << std::setprecision(6);
    for (const BenchmarkRow& row : rows) {
        csv << row.caseName << ','
            << row.partCount << ','
            << profileName(row.profile) << ','
            << row.elapsedMs << ','
            << row.utilization << ','
            << row.usedWidth << ','
            << row.usedHeight << ','
            << row.collisions << ','
            << row.invalid << ','
            << row.spacingPenalty << ','
            << row.sheetPenalty << ','
            << row.bestUpdates << ','
            << row.evaluatedCandidates << ','
            << row.candidatesPerSecond << ','
            << row.acceptedMoves << ','
            << row.acceptedWorseMoves << ','
            << row.gapAccepted << ','
            << row.swapAccepted << ','
            << row.chainAccepted << ','
            << row.escapeAccepted << ','
            << row.ultraAccepted << ','
            << row.compactionAccepted << ','
            << row.frontierCandidates << ','
            << row.smallFillerAccepted << ','
            << row.regionRepackAccepted << ','
            << row.finalScore << ','
            << (row.profilePass ? "PASS" : "FAIL") << '\n';
    }
}

void printHumanRow(const BenchmarkRow& row) {
    std::cout << (row.profilePass ? "PASS" : "FAIL")
        << " case=" << row.caseName
        << " profile=" << profileName(row.profile)
        << " parts=" << row.partCount
        << " elapsedMs=" << static_cast<long long>(std::llround(row.elapsedMs))
        << " util=" << std::fixed << std::setprecision(4) << row.utilization
        << " score=" << std::setprecision(2) << row.finalScore
        << " collisions=" << row.collisions
        << " invalid=" << row.invalid
        << " spacingPenalty=" << std::setprecision(6) << row.spacingPenalty
        << " sheetPenalty=" << row.sheetPenalty
        << " bestUpdates=" << row.bestUpdates
        << " cps=" << std::setprecision(1) << row.candidatesPerSecond;
    if (!row.failureReason.empty()) {
        std::cout << " reason=" << row.failureReason;
    }
    std::cout << '\n';
}

LayoutState solveOnce(const BenchmarkCase& benchmarkCase, PerformanceProfile profile, const std::filesystem::path& root) {
    const std::filesystem::path inputFile = root / L"samples" / L"benchmark" / benchmarkCase.fileName;
    EngineSettings preliminary = settingsFor(benchmarkCase, 1, profile);
    bool importOk = false;
    std::wstring importMessage;
    Document document = buildDocument(benchmarkCase, inputFile, preliminary, importOk, importMessage);
    if (!importOk) {
        return {};
    }
    EngineSettings settings = settingsFor(benchmarkCase, document.parts.size(), profile);
    document.sheet.width = settings.sheetWidth;
    document.sheet.height = settings.sheetHeight;
    document.sheet.margin = settings.margin;
    std::atomic_bool stopRequested{false};
    MultiStartSolver solver;
    LayoutState solved = solver.solve(document, settings, stopRequested, {});
    LayoutScore scorer;
    PenaltySystem penalties;
    return scorer.evaluate(document, settings, solved.poses, &penalties);
}

bool deterministicCheck(const BenchmarkCase& benchmarkCase, const std::filesystem::path& root) {
    const LayoutState first = solveOnce(benchmarkCase, PerformanceProfile::Balanced, root);
    const LayoutState second = solveOnce(benchmarkCase, PerformanceProfile::Balanced, root);
    return first.valid() && second.valid() && layoutAlmostEqual(first, second);
}

BenchmarkRow* findRow(std::vector<BenchmarkRow>& rows, const std::string& caseName, PerformanceProfile profile) {
    for (BenchmarkRow& row : rows) {
        if (row.caseName == caseName && row.profile == profile) {
            return &row;
        }
    }
    return nullptr;
}

const BenchmarkRow* findRow(const std::vector<BenchmarkRow>& rows, const std::string& caseName, PerformanceProfile profile) {
    for (const BenchmarkRow& row : rows) {
        if (row.caseName == caseName && row.profile == profile) {
            return &row;
        }
    }
    return nullptr;
}

bool applyQualityGoal(std::vector<BenchmarkRow>& rows, const std::string& caseName, bool condition, const char* message) {
    if (condition) {
        std::cout << "PASS quality_goal case=" << caseName << " rule=" << message << "\n";
        return true;
    }
    BenchmarkRow* maximum = findRow(rows, caseName, PerformanceProfile::Maximum);
    if (maximum) {
        maximum->profilePass = false;
        maximum->failureReason = message;
    }
    std::cout << "FAIL quality_goal case=" << caseName << " rule=" << message << "\n";
    return false;
}

std::vector<BenchmarkCase> benchmarkCases() {
    return {
        {"many_small_parts", L"many_small_parts.svg", 1150.0, 760.0, 8.0, 2.0, false, false, false, RotationMode::None, 90.0, PlacementStrategy::BottomLeft, 101u},
        {"mixed_100_parts", L"mixed_100_parts.svg", 1450.0, 940.0, 10.0, 2.0, false, false, false, RotationMode::None, 90.0, PlacementStrategy::BottomLeft, 102u},
        {"mixed_250_parts", L"mixed_250_parts.svg", 2250.0, 1450.0, 10.0, 2.0, false, false, false, RotationMode::None, 90.0, PlacementStrategy::BottomLeft, 103u},
        {"mixed_500_parts", L"mixed_500_parts.svg", 3300.0, 2150.0, 12.0, 2.0, false, false, false, RotationMode::None, 90.0, PlacementStrategy::BottomLeft, 104u},
        {"donut_hole_usage", L"donut_hole_usage.svg", 520.0, 320.0, 8.0, 3.0, false, true, false, RotationMode::RightAngles, 90.0, PlacementStrategy::CenterOut, 105u},
        {"b_shape_hole_usage", L"b_shape_hole_usage.svg", 560.0, 360.0, 8.0, 3.0, false, true, true, RotationMode::RightAngles, 90.0, PlacementStrategy::CenterOut, 106u},
        {"concave_cavity_usage", L"concave_cavity_usage.svg", 620.0, 380.0, 8.0, 3.0, false, true, true, RotationMode::FortyFiveDegrees, 45.0, PlacementStrategy::BottomLeft, 107u},
        {"irregular_sheet_with_hole", L"irregular_sheet_with_hole.svg", 820.0, 740.0, 0.0, 2.0, true, false, false, RotationMode::None, 90.0, PlacementStrategy::BottomLeft, 108u},
        {"mirror_required", L"mirror_required.svg", 760.0, 420.0, 8.0, 2.0, false, true, true, RotationMode::FortyFiveDegrees, 45.0, PlacementStrategy::BottomLeft, 109u},
        {"rotation_precision_required", L"rotation_precision_required.svg", 700.0, 420.0, 8.0, 2.0, false, true, false, RotationMode::ContinuousRefine, 0.001, PlacementStrategy::BottomLeft, 110u}
    };
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    const std::filesystem::path root = argc > 1 ? std::filesystem::path(argv[1]) : std::filesystem::current_path();
    const std::filesystem::path outputDir = root / L"outputs" / L"benchmark";
    std::filesystem::create_directories(outputDir);

    const std::vector<BenchmarkCase> cases = benchmarkCases();
    const PerformanceProfile profiles[] = {
        PerformanceProfile::Fast,
        PerformanceProfile::Balanced,
        PerformanceProfile::Maximum
    };

    std::vector<BenchmarkRow> rows;
    rows.reserve(cases.size() * 3);
    bool allPass = true;

    std::cout << "CigerNesting benchmark runner\n";
    std::cout << "root=" << narrowPath(root) << "\n";
    std::cout << "output=" << narrowPath(outputDir) << "\n";

    for (const BenchmarkCase& benchmarkCase : cases) {
        double fastScore = std::numeric_limits<double>::quiet_NaN();
        size_t fastIndex = rows.size();
        for (PerformanceProfile profile : profiles) {
            BenchmarkRow row = runBenchmark(benchmarkCase, profile, root, outputDir);
            if (profile == PerformanceProfile::Fast) {
                fastScore = row.finalScore;
                fastIndex = rows.size();
            }
            if (profile == PerformanceProfile::Maximum && std::isfinite(fastScore) && !scoreNoWorse(row.finalScore, fastScore)) {
                row.profilePass = false;
                row.failureReason = "Maximum score worse than Fast";
            }
            allPass = row.profilePass && allPass;
            printHumanRow(row);
            rows.push_back(std::move(row));
        }
        if (fastIndex < rows.size() && !rows[fastIndex].qualityPass) {
            allPass = false;
        }
    }

    const BenchmarkRow* manyFast = findRow(rows, "many_small_parts", PerformanceProfile::Fast);
    const BenchmarkRow* manyMaximum = findRow(rows, "many_small_parts", PerformanceProfile::Maximum);
    if (manyFast && manyMaximum) {
        const double fastArea = manyFast->usedWidth * manyFast->usedHeight;
        const double maximumArea = manyMaximum->usedWidth * manyMaximum->usedHeight;
        allPass = applyQualityGoal(rows, "many_small_parts", manyMaximum->bestUpdates > 1, "Maximum bestUpdates > 1") && allPass;
        allPass = applyQualityGoal(rows, "many_small_parts",
            manyMaximum->utilization > manyFast->utilization + 1e-6 || maximumArea + 1e-6 < fastArea,
            "Maximum improves utilization or used area vs Fast") && allPass;
    }

    const BenchmarkRow* mixed100Maximum = findRow(rows, "mixed_100_parts", PerformanceProfile::Maximum);
    if (mixed100Maximum) {
        allPass = applyQualityGoal(rows, "mixed_100_parts", mixed100Maximum->bestUpdates > 0, "Maximum bestUpdates > 0") && allPass;
    }

    const BenchmarkRow* mixed500Maximum = findRow(rows, "mixed_500_parts", PerformanceProfile::Maximum);
    if (mixed500Maximum) {
        allPass = applyQualityGoal(rows, "mixed_500_parts", mixed500Maximum->bestUpdates > 0, "Maximum bestUpdates > 0") && allPass;
    }

    const BenchmarkCase* mirrorCase = nullptr;
    for (const BenchmarkCase& benchmarkCase : cases) {
        if (benchmarkCase.name == "mirror_required") {
            mirrorCase = &benchmarkCase;
            break;
        }
    }
    const BenchmarkRow* mirrorEnabled = findRow(rows, "mirror_required", PerformanceProfile::Balanced);
    if (mirrorCase && mirrorEnabled) {
        BenchmarkCase disabled = *mirrorCase;
        disabled.name = "mirror_required_disabled";
        disabled.allowMirroring = false;
        BenchmarkRow disabledRow = runBenchmark(disabled, PerformanceProfile::Balanced, root, outputDir);
        const bool mirrorPass = disabledRow.qualityPass && mirrorEnabled->qualityPass &&
            scoreNoWorse(mirrorEnabled->finalScore, disabledRow.finalScore);
        std::cout << (mirrorPass ? "PASS" : "FAIL")
                  << " mirror_comparison disabledScore=" << disabledRow.finalScore
                  << " enabledScore=" << mirrorEnabled->finalScore << "\n";
        rows.push_back(std::move(disabledRow));
        allPass = mirrorPass && allPass;
    }

    const bool deterministicPass = deterministicCheck(cases.front(), root);
    allPass = deterministicPass && allPass;
    std::cout << (deterministicPass ? "PASS" : "FAIL") << " deterministic_seed case=" << cases.front().name << " profile=Balanced\n";

    const std::filesystem::path csvPath = outputDir / L"benchmark_results.csv";
    writeCsv(csvPath, rows);
    std::cout << "csv=" << narrowPath(csvPath) << "\n";
    std::cout << "json_svg_dir=" << narrowPath(outputDir) << "\n";
    return allPass ? 0 : 1;
}
