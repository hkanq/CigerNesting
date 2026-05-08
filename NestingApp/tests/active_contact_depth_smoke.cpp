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
    ok = expect("Maximum active contact target includes 24-depth tier", source.find("24u") != std::string::npos) && ok;
    ok = expect("Maximum active contact target includes 34-depth tier", source.find("34u") != std::string::npos) && ok;
    ok = expect("active contact depth stats are recorded", source.find("averageActiveContactDepth") != std::string::npos) && ok;
    ok = expect("old shallow active cap variable is gone", source.find("const size_t cap = partCount >= 400 ? 2u : 3u") == std::string::npos) && ok;
    return ok ? 0 : 1;
}
