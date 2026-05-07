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
    const std::string source = readText(repoRoot(argc, argv) / "NestingApp" / "engine" / "constructive_rebuild_engine.cpp");
    bool ok = true;
    ok = expect("Maximum attempts are in the industrial rebuild range", source.find("45u") != std::string::npos && source.find("60u") != std::string::npos) && ok;
    ok = expect("Maximum beam width is at least 24", source.find("24u") != std::string::npos && source.find("32u") != std::string::npos) && ok;
    ok = expect("placement depth records real subset depth", source.find("placementDepthTotal") != std::string::npos && source.find("averagePlacementDepth") != std::string::npos) && ok;
    ok = expect("valid leaf limit is explicit", source.find("validLeafLimit") != std::string::npos) && ok;
    const std::string fabricatedDestroyAttempts = std::string("stats.") + "destroyAttempts = 30";
    ok = expect("test does not fabricate solver stats", source.find(fabricatedDestroyAttempts) == std::string::npos) && ok;
    return ok ? 0 : 1;
}
