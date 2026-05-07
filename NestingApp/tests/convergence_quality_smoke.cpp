#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

std::string readText(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
}

bool expect(const char* name, bool condition) {
    std::cout << (condition ? "PASS: " : "FAIL: ") << name << "\n";
    return condition;
}

std::filesystem::path repoRoot(int argc, char** argv) {
    if (argc > 1) {
        return std::filesystem::path(argv[1]);
    }
    std::filesystem::path current = std::filesystem::current_path();
    for (int i = 0; i < 6; ++i) {
        if (std::filesystem::exists(current / "NestingApp" / "engine" / "constructive_rebuild_engine.cpp")) {
            return current;
        }
        current = current.parent_path();
    }
    return std::filesystem::current_path();
}

} // namespace

int main(int argc, char** argv) {
    const std::filesystem::path root = repoRoot(argc, argv);
    const std::string constructive = readText(root / "NestingApp" / "engine" / "constructive_rebuild_engine.cpp");
    const std::string solver = readText(root / "NestingApp" / "engine" / "multi_start_solver.cpp");

    bool ok = true;
    ok = expect("constructive rebuild publishes in-attempt preview events", constructive.find("rebuildPreviewEvents") != std::string::npos) && ok;
    ok = expect("convergence is not represented by fake accepted move counters", constructive.find("acceptedMoves = 184") == std::string::npos && solver.find("acceptedMoves = 184") == std::string::npos) && ok;
    ok = expect("constructive rebuild tracks empty region objective", constructive.find("largestRegionArea") != std::string::npos && constructive.find("totalEmptyArea") != std::string::npos) && ok;
    ok = expect("constructive rebuild no longer hard-caps placement depth at four", constructive.find("min<size_t>(subsetSize, 4u)") == std::string::npos) && ok;
    return ok ? 0 : 1;
}
