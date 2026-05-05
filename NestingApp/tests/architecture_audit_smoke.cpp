#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

std::string readText(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream out;
    out << in.rdbuf();
    return out.str();
}

bool contains(const std::string& text, const char* needle) {
    return text.find(needle) != std::string::npos;
}

int fail(const char* message) {
    std::cout << "FAIL: " << message << "\n";
    return 1;
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    if (argc < 2) {
        std::wcerr << L"usage: architecture_audit_smoke <repo-root>\n";
        return 2;
    }
    const std::filesystem::path root = argv[1];
    const std::string optimizer = readText(root / L"NestingApp" / L"engine" / L"adaptive_optimizer.cpp");
    const std::string solver = readText(root / L"NestingApp" / L"engine" / L"multi_start_solver.cpp");
    const std::string mainWindow = readText(root / L"NestingApp" / L"app" / L"main_window.cpp");
    const std::string canvas = readText(root / L"NestingApp" / L"ui" / L"canvas_view.cpp");

    if (!contains(optimizer, "struct OperatorContext") ||
        !contains(optimizer, "buildMoveTasks") ||
        !contains(optimizer, "needsContactPacking")) {
        return fail("adaptive optimizer does not expose part-need task scheduling");
    }
    if (!contains(solver, "contourSeedBaseline") ||
        !contains(solver, "settings.performanceProfile != PerformanceProfile::Fast")) {
        return fail("Balanced/Maximum contour seed path is missing");
    }
    if (contains(solver, "Compression compression;") ||
        contains(solver, "GapFilling gap") ||
        contains(solver, "Rearrangement rearrangement") ||
        contains(solver, "UltraRefinement ultra")) {
        return fail("old phase pipeline is still directly called by MultiStartSolver");
    }
    if (contains(mainWindow, "textIdForStrategy(snapshot.currentStrategy))") ||
        contains(canvas, "textIdForStrategy(snapshot.currentStrategy))")) {
        return fail("UI still falls back to a single currentStrategy label");
    }

    std::cout << "PASS: architecture audit smoke\n";
    return 0;
}
