#pragma once

#include "engine/engine_settings.h"
#include <cstddef>

namespace nest {

struct AcceptanceDecision {
    bool accepted = false;
    bool acceptedWorse = false;
    double probability = 0.0;
    double temperature = 0.0;
};

class AcceptanceCriteria {
public:
    explicit AcceptanceCriteria(const EngineSettings& settings);

    AcceptanceDecision decide(
        double currentScore,
        double candidateScore,
        int iteration,
        int maxIterations,
        unsigned int seed,
        size_t candidateIndex) const;

    double temperature(double scoreScale, int iteration, int maxIterations) const;

private:
    double initialFactor_ = 0.01;
    double finalFactor_ = 0.0005;
};

} // namespace nest
