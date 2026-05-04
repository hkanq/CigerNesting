#include "engine/acceptance.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace nest {
namespace {

double random01(unsigned int seed, size_t candidateIndex, int iteration) {
    uint64_t value = static_cast<uint64_t>(seed) + 0x9e3779b97f4a7c15ull;
    value ^= static_cast<uint64_t>(candidateIndex + 1u) * 0xbf58476d1ce4e5b9ull;
    value ^= static_cast<uint64_t>(iteration + 17) * 0x94d049bb133111ebull;
    value ^= value >> 30;
    value *= 0xbf58476d1ce4e5b9ull;
    value ^= value >> 27;
    value *= 0x94d049bb133111ebull;
    value ^= value >> 31;
    constexpr double denominator = static_cast<double>(uint64_t{1} << 53);
    return static_cast<double>((value >> 11) & ((uint64_t{1} << 53) - 1u)) / denominator;
}

} // namespace

AcceptanceCriteria::AcceptanceCriteria(const EngineSettings& settings) {
    switch (settings.performanceProfile) {
    case PerformanceProfile::Fast:
        initialFactor_ = 0.0015;
        finalFactor_ = 0.00005;
        break;
    case PerformanceProfile::Maximum:
        initialFactor_ = 0.035;
        finalFactor_ = 0.0015;
        break;
    case PerformanceProfile::Balanced:
    default:
        initialFactor_ = 0.010;
        finalFactor_ = 0.0005;
        break;
    }
}

double AcceptanceCriteria::temperature(double scoreScale, int iteration, int maxIterations) const {
    const double progress = maxIterations <= 1 ? 1.0 :
        std::clamp(static_cast<double>(iteration) / static_cast<double>(maxIterations - 1), 0.0, 1.0);
    const double factor = initialFactor_ * std::pow(finalFactor_ / std::max(initialFactor_, 1e-12), progress);
    return std::max(1.0, std::abs(scoreScale) * factor);
}

AcceptanceDecision AcceptanceCriteria::decide(
    double currentScore,
    double candidateScore,
    int iteration,
    int maxIterations,
    unsigned int seed,
    size_t candidateIndex) const {
    AcceptanceDecision decision;
    const double delta = candidateScore - currentScore;
    decision.temperature = temperature(std::max(std::abs(currentScore), std::abs(candidateScore)), iteration, maxIterations);
    if (delta <= 0.0) {
        decision.accepted = true;
        decision.probability = 1.0;
        return decision;
    }

    decision.probability = std::exp(-delta / std::max(1e-9, decision.temperature));
    decision.accepted = random01(seed, candidateIndex, iteration) < decision.probability;
    decision.acceptedWorse = decision.accepted;
    return decision;
}

} // namespace nest
