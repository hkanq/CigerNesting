#pragma once

#include "engine/engine_settings.h"
#include <vector>

namespace nest {

class PoseSampler {
public:
    std::vector<double> coarseRotationSamples(const EngineSettings& settings) const;
    std::vector<double> refinementRotationSamples(double centerRadians, double stepDegrees, int radius) const;
    std::vector<bool> mirrorSamples(const EngineSettings& settings) const;
};

} // namespace nest
