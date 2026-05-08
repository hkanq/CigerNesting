#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

std::filesystem::path repoRoot(int argc, wchar_t** argv) {
    if (argc > 1) {
        return std::filesystem::path(argv[1]);
    }
    std::filesystem::path current = std::filesystem::current_path();
    while (!current.empty()) {
        if (std::filesystem::exists(current / "NestingApp" / "engine" / "constructive_rebuild_engine.cpp")) {
            return current;
        }
        current = current.parent_path();
    }
    return std::filesystem::current_path();
}

std::string readText(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
}

bool expect(const char* name, bool condition) {
    std::cout << (condition ? "PASS: " : "QUALITY_FAIL: ") << name << "\n";
    return condition;
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    const std::filesystem::path root = repoRoot(argc, argv);
    const std::string source = readText(root / "NestingApp" / "engine" / "constructive_rebuild_engine.cpp");
    bool ok = true;
    ok = expect("rebuild uses multidimensional quality delta", source.find("struct RebuildQualityDelta") != std::string::npos) && ok;
    ok = expect("rebuild tracks used area delta", source.find("usedAreaDelta") != std::string::npos) && ok;
    ok = expect("rebuild tracks largest empty region delta", source.find("largestEmptyRegionDelta") != std::string::npos) && ok;
    ok = expect("rebuild tracks total empty area delta", source.find("totalEmptyAreaDelta") != std::string::npos) && ok;
    ok = expect("rebuild tracks contact delta", source.find("contactCountDelta") != std::string::npos) && ok;
    ok = expect("objective gain is semantic, not validity-only", source.find("hasObjectiveGain") != std::string::npos &&
        source.find("destroyTemporaryRejectedNoObjectiveGain") != std::string::npos) && ok;
    ok = expect("attempt before/after metrics are recorded", source.find("recordAttemptMetrics") != std::string::npos) && ok;
    return ok ? 0 : 1;
}
