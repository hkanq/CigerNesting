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
        if (std::filesystem::exists(current / "NestingApp" / "engine" / "multi_start_solver.cpp")) {
            return current;
        }
        current = current.parent_path();
    }
    return std::filesystem::current_path();
}

} // namespace

int main(int argc, char** argv) {
    const std::string source = readText(repoRoot(argc, argv) / "NestingApp" / "engine" / "multi_start_solver.cpp");
    bool ok = true;
    ok = expect("contour seed has strategy-specific anchors", source.find("strategySheetAnchors") != std::string::npos) && ok;
    ok = expect("BottomLeft anchor is explicitly first-capable", source.find("appendUniqueAnchor(anchors, bottomLeft)") != std::string::npos) && ok;
    ok = expect("TopLeft anchor is explicitly first-capable", source.find("appendUniqueAnchor(anchors, topLeft)") != std::string::npos) && ok;
    ok = expect("BottomRight anchor is explicitly first-capable", source.find("appendUniqueAnchor(anchors, bottomRight)") != std::string::npos) && ok;
    ok = expect("TopRight anchor is explicitly first-capable", source.find("appendUniqueAnchor(anchors, topRight)") != std::string::npos) && ok;
    ok = expect("placement strategy affects contour seed score", source.find("placementStrategyPenalty(settings, used)") != std::string::npos) && ok;
    return ok ? 0 : 1;
}
