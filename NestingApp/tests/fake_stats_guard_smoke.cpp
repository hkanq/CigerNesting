#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string readText(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
}

std::filesystem::path repoRoot(int argc, char** argv) {
    if (argc > 1) {
        return std::filesystem::path(argv[1]);
    }
    std::filesystem::path current = std::filesystem::current_path();
    for (int i = 0; i < 6; ++i) {
        if (std::filesystem::exists(current / "NestingApp" / "tests")) {
            return current;
        }
        current = current.parent_path();
    }
    return std::filesystem::current_path();
}

bool suspiciousManualStats(const std::string& text) {
    const char* patterns[] = {
        "stats.destroyAttempts =",
        "stats.destroyAccepted =",
        "stats.averageSubsetSize =",
        "stats.beamNodesExpanded =",
        "stats.beamValidLeaves =",
        "stats.largestEmptyRegionArea =",
        "stats.fillableGapCount =",
        "stats.acceptedMoves = 184"
    };
    for (const char* pattern : patterns) {
        if (text.find(pattern) != std::string::npos) {
            return true;
        }
    }
    return false;
}

} // namespace

int main(int argc, char** argv) {
    const std::filesystem::path tests = repoRoot(argc, argv) / "NestingApp" / "tests";
    bool ok = true;
    for (const auto& entry : std::filesystem::directory_iterator(tests)) {
        if (!entry.is_regular_file() || entry.path().extension() != ".cpp") {
            continue;
        }
        const std::string name = entry.path().filename().string();
        if (name == "fake_stats_guard_smoke.cpp" || name == "benchmark_runner.cpp" || name == "performance_stress_smoke.cpp") {
            continue;
        }
        if (suspiciousManualStats(readText(entry.path()))) {
            std::cout << "FAIL: suspicious manual SolverStats assignment in " << name << "\n";
            ok = false;
        }
    }
    if (ok) {
        std::cout << "PASS: no fake SolverStats smoke shortcuts found\n";
    }
    return ok ? 0 : 1;
}
