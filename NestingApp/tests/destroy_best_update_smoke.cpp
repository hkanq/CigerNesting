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
        if (std::filesystem::exists(current / "NestingApp" / "tests" / "constructive_rebuild_smoke.cpp")) {
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
    const std::string constructiveSmoke = readText(root / "NestingApp" / "tests" / "constructive_rebuild_smoke.cpp");
    const std::string destroySmoke = readText(root / "NestingApp" / "tests" / "destroy_rebuild_smoke.cpp");
    bool ok = true;
    ok = expect("constructive smoke reports destroy best updates", constructiveSmoke.find("destroyBestUpdates") != std::string::npos) && ok;
    ok = expect("constructive smoke does not treat destroyAccepted alone as success",
        constructiveSmoke.find("destroyAccepted > 0") == std::string::npos) && ok;
    ok = expect("constructive smoke requires used-area/utilization quality signal",
        constructiveSmoke.find("bestRebuildUsedAreaReduction > 1.0") != std::string::npos &&
        constructiveSmoke.find("bestRebuildUtilizationGain > 0.010") != std::string::npos) && ok;
    ok = expect("destroy smoke blocks fake accepted trajectory success",
        destroySmoke.find("destroyAccepted == stats.destroyBestUpdates + stats.destroyTemporaryAcceptedWithObjectiveGain") != std::string::npos &&
        destroySmoke.find("bestRebuildUsedAreaReduction > 1.0") != std::string::npos &&
        destroySmoke.find("bestRebuildUtilizationGain > 0.010") != std::string::npos) && ok;
    return ok ? 0 : 1;
}
