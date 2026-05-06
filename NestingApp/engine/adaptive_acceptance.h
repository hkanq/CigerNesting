#pragma once

#include "engine/engine_settings.h"
#include <cstddef>

namespace nest {

struct AdaptiveAcceptanceContext {
    double currentScore = 0.0;
    double candidateScore = 0.0;
    double bestScore = 0.0;
    double emptySpacePotential = 0.0;
    double contactPotential = 0.0;
    bool destroyRebuildMove = false;
    int iteration = 0;
    int maxIterations = 1;
    unsigned int seed = 1u;
    size_t candidateIndex = 0;
};

struct AdaptiveAcceptanceDecision {
    bool accepted = false;
    bool better = false;
    bool temporary = false;
    bool acceptedWorse = false;
    double probability = 0.0;
    double temperature = 0.0;
};

class AdaptiveAcceptance {
public:
    explicit AdaptiveAcceptance(const EngineSettings& settings);

    AdaptiveAcceptanceDecision decide(const AdaptiveAcceptanceContext& context) const;

private:
    double initialFactor_ = 0.015;
    double finalFactor_ = 0.0005;
    double maxWorseRatio_ = 0.015;
};

} // namespace nest
