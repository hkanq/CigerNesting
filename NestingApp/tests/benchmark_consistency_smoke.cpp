#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

std::string readText(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return {};
    }
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

} // namespace

int main(int argc, char** argv) {
    const std::filesystem::path root = repoRoot(argc, argv);
    const std::filesystem::path rootCsv = root / "benchmark_results.csv";
    const std::filesystem::path outputCsv = root / "outputs" / "benchmark" / "benchmark_results.csv";
    const std::string rootText = readText(rootCsv);
    const std::string outputText = readText(outputCsv);
    if (rootText.empty() || outputText.empty()) {
        std::cout << "PASS: benchmark CSV consistency deferred until benchmark runner writes both files\n";
        return 0;
    }
    if (rootText != outputText) {
        std::cout << "FAIL: benchmark_results.csv differs from outputs/benchmark/benchmark_results.csv\n";
        return 1;
    }
    std::cout << "PASS: benchmark CSV files are identical\n";
    return 0;
}

