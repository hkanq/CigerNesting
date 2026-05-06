#include "engine/adaptive_acceptance.h"

#include <iostream>

namespace {

using namespace nest;

bool expect(const char* name, bool condition) {
    std::cout << (condition ? "PASS: " : "FAIL: ") << name << "\n";
    return condition;
}

EngineSettings maximumSettings() {
    EngineSettings settings;
    settings.performanceProfile = PerformanceProfile::Maximum;
    settings.qualityMode = QualityMode::MaxQuality;
    settings.deterministic = true;
    settings.randomSeed = 42u;
    return settings;
}

} // namespace

int main() {
    EngineSettings settings = maximumSettings();
    AdaptiveAcceptance acceptance(settings);
    bool ok = true;

    AdaptiveAcceptanceContext better;
    better.currentScore = 10000.0;
    better.candidateScore = 9900.0;
    better.bestScore = 9800.0;
    const AdaptiveAcceptanceDecision betterDecision = acceptance.decide(better);
    ok = expect("better candidate is always accepted", betterDecision.accepted && betterDecision.better) && ok;

    AdaptiveAcceptanceContext promisingWorse;
    promisingWorse.currentScore = 10000.0;
    promisingWorse.candidateScore = 10100.0;
    promisingWorse.bestScore = 9800.0;
    promisingWorse.emptySpacePotential = 0.65;
    promisingWorse.contactPotential = 0.45;
    promisingWorse.destroyRebuildMove = true;
    promisingWorse.iteration = 2;
    promisingWorse.maxIterations = 80;
    promisingWorse.seed = 7u;
    promisingWorse.candidateIndex = 3u;
    const AdaptiveAcceptanceDecision worseDecision = acceptance.decide(promisingWorse);
    std::cout << "promisingWorse"
              << " accepted=" << worseDecision.accepted
              << " probability=" << worseDecision.probability
              << " temperature=" << worseDecision.temperature
              << "\n";
    ok = expect("promising worse valid candidate can be temporary accepted", worseDecision.accepted && worseDecision.temporary) && ok;

    AdaptiveAcceptanceContext destructiveBad = promisingWorse;
    destructiveBad.candidateScore = 12500.0;
    destructiveBad.candidateIndex = 19u;
    const AdaptiveAcceptanceDecision badDecision = acceptance.decide(destructiveBad);
    ok = expect("large score regression is rejected", !badDecision.accepted) && ok;

    return ok ? 0 : 1;
}
