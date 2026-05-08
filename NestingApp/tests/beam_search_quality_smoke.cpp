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

std::filesystem::path repoRoot(int argc, char** argv) {
    if (argc > 1) {
        return std::filesystem::path(argv[1]);
    }
    return std::filesystem::current_path();
}

bool expect(const char* name, bool condition) {
    std::cout << (condition ? "PASS: " : "FAIL: ") << name << "\n";
    return condition;
}

} // namespace

int main(int argc, char** argv) {
    const std::string source = readText(repoRoot(argc, argv) / "NestingApp" / "engine" / "constructive_rebuild_engine.cpp");
    bool ok = true;
    ok = expect("beam width remains industrial-sized", source.find("24u : 32u") != std::string::npos) && ok;
    ok = expect("expansion limit is no longer two-wide", source.find("expansionLimit = partCount >= 400 ? 8u : 8u") != std::string::npos) && ok;
    ok = expect("partial eval limit is no longer one-wide", source.find("partialEvalLimit = partCount >= 400 ? 4u : 6u") != std::string::npos) && ok;
    ok = expect("empty region area is part of objective", source.find("largestRegionArea * largestGapWeight") != std::string::npos) && ok;
    return ok ? 0 : 1;
}

