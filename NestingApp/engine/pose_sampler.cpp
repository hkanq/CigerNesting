#include "engine/pose_sampler.h"

#include "core/math_utils.h"
#include <algorithm>
#include <cmath>

namespace nest {

std::vector<double> PoseSampler::coarseRotationSamples(const EngineSettings& settings) const {
    if (!settings.allowRotation || settings.rotationMode == RotationMode::None) {
        return {0.0};
    }

    double step = 90.0;
    if (settings.rotationMode == RotationMode::FortyFiveDegrees) {
        step = 45.0;
    } else if (settings.rotationMode == RotationMode::FixedStep) {
        step = std::max(0.001, settings.rotationStepDegrees);
        step = std::max(step, 5.0); // FixedStep enters fine values later; coarse pass stays bounded.
    } else if (settings.rotationMode == RotationMode::ContinuousRefine) {
        step = 30.0;
    }

    std::vector<double> samples;
    for (double degrees = 0.0; degrees < 360.0 - 1e-9; degrees += step) {
        samples.push_back(degreesToRadians(degrees));
    }
    if (samples.empty()) {
        samples.push_back(0.0);
    }
    return samples;
}

std::vector<double> PoseSampler::refinementRotationSamples(double centerRadians, double stepDegrees, int radius) const {
    const double step = degreesToRadians(std::max(0.001, stepDegrees));
    std::vector<double> samples;
    for (int i = -radius; i <= radius; ++i) {
        samples.push_back(centerRadians + static_cast<double>(i) * step);
    }
    return samples;
}

std::vector<bool> PoseSampler::mirrorSamples(const EngineSettings& settings) const {
    return settings.allowMirroring ? std::vector<bool>{false, true} : std::vector<bool>{false};
}

} // namespace nest
