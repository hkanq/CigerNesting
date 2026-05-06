#include "engine/adaptive_acceptance.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace nest {
namespace {

double random01(unsigned int seed, size_t candidateIndex, int iteration) {
    uint64_t value = static_cast<uint64_t>(seed) + 0x9e3779b97f4a7c15ull;
    value ^= static_cast<uint64_t>(candidateIndex + 1u) * 0xbf58476d1ce4e5b9ull;
    value ^= static_cast<uint64_t>(iteration + 29) * 0x94d049bb133111ebull;
    value ^= value >> 30;
    value *= 0xbf58476d1ce4e5b9ull;
    value ^= value >> 27;
    value *= 0x94d049bb133111ebull;
    value ^= value >> 31;
    constexpr double denominator = static_cast<double>(uint64_t{1} << 53);
    return static_cast<double>((value >> 11) & ((uint64_t{1} << 53) - 1u)) / denominator;
}

} // namespace

AdaptiveAcceptance::AdaptiveAcceptance(const EngineSettings& settings) {
    switch (settings.performanceProfile) {
    case PerformanceProfile::Fast:
        initialFactor_ = 0.0015;
        finalFactor_ = 0.00004;
        maxWorseRatio_ = 0.003;
        break;
    case PerformanceProfile::Maximum:
        initialFactor_ = 0.160;
        finalFactor_ = 0.0080;
        maxWorseRatio_ = 0.220;
        break;
    case PerformanceProfile::Balanced:
    default:
        initialFactor_ = 0.070;
        finalFactor_ = 0.0025;
        maxWorseRatio_ = 0.080;
        break;
    }
}

AdaptiveAcceptanceDecision AdaptiveAcceptance::decide(const AdaptiveAcceptanceContext& context) const {
    AdaptiveAcceptanceDecision decision;
    const double delta = context.candidateScore - context.currentScore;
    if (delta <= 0.0) {
        decision.accepted = true;
        decision.better = true;
        decision.probability = 1.0;
        return decision;
    }

    const double progress = context.maxIterations <= 1
        ? 1.0
        : std::clamp(static_cast<double>(context.iteration) / static_cast<double>(context.maxIterations - 1), 0.0, 1.0);
    const double potentialBoost = 1.0 +
        std::clamp(context.emptySpacePotential, 0.0, 1.5) * 1.8 +
        std::clamp(context.contactPotential, 0.0, 1.5) * 0.8 +
        (context.destroyRebuildMove ? 1.25 : 0.0);
    const double factor = initialFactor_ * std::pow(finalFactor_ / std::max(initialFactor_, 1e-12), progress);
    const double scale = std::max({1.0, std::abs(context.currentScore), std::abs(context.bestScore)});
    decision.temperature = std::max(1.0, scale * factor * potentialBoost);
    const double worseRatio = delta / scale;
    const double allowedWorse = maxWorseRatio_ * potentialBoost;
    if (worseRatio > allowedWorse) {
        decision.probability = 0.0;
        return decision;
    }

    decision.probability = std::exp(-delta / std::max(1e-9, decision.temperature));
    decision.accepted = random01(context.seed, context.candidateIndex, context.iteration) < decision.probability;
    decision.temporary = decision.accepted;
    decision.acceptedWorse = decision.accepted;
    return decision;
}

} // namespace nest
