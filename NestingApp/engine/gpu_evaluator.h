#pragma once

namespace nest {

class IGpuEvaluator {
public:
    virtual ~IGpuEvaluator() = default;

    // Future extension point: GPU work should stay limited to candidate pose scoring,
    // broadphase acceleration, or raster occupancy estimates. Solver ownership remains CPU-side.
    virtual bool available() const = 0;
};

} // namespace nest
