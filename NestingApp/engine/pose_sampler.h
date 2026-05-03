#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include <vector>

namespace nest {

class PoseSampler {
public:
    std::vector<double> coarseRotationSamples(const EngineSettings& settings) const;
    std::vector<double> refinementRotationSamples(double centerRadians, double stepDegrees, int radius) const;
    std::vector<bool> mirrorSamples(const EngineSettings& settings) const;
    std::vector<Pose> moveCandidates(const Document& document, const EngineSettings& settings, const std::vector<Pose>& poses, size_t partIndex, unsigned int seed, int iteration) const;
};

} // namespace nest
