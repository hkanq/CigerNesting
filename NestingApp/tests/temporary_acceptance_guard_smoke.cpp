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
    const std::string stats = readText(root / "NestingApp" / "engine" / "solver_state.h");
    bool ok = true;
    ok = expect("temporary accepted with objective gain is tracked", stats.find("destroyTemporaryAcceptedWithObjectiveGain") != std::string::npos) && ok;
    ok = expect("temporary rejected without objective gain is tracked", stats.find("destroyTemporaryRejectedNoObjectiveGain") != std::string::npos) && ok;
    ok = expect("valid-only maximum temporary acceptance is removed",
        source.find("settings.performanceProfile == PerformanceProfile::Maximum && bestAttempt.state.valid()") == std::string::npos &&
        source.find("accepted = true;\r\n            temporaryAccepted = true;") == std::string::npos) && ok;
    ok = expect("temporary acceptance checks objective gain before annealing",
        source.find("if (!hasObjectiveGain(currentDelta))") != std::string::npos &&
        source.find("destroyTemporaryRejectedNoObjectiveGain") != std::string::npos) && ok;
    return ok ? 0 : 1;
}
