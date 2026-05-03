#include "engine/penalty_system.h"

#include <algorithm>

namespace nest {

void PenaltySystem::reset() {
    weights_.clear();
}

std::pair<size_t, size_t> PenaltySystem::key(size_t a, size_t b) {
    return {std::min(a, b), std::max(a, b)};
}

void PenaltySystem::observeCollision(size_t a, size_t b) {
    auto& value = weights_[key(a, b)];
    value = value <= 0.0 ? 1.0 : std::min(32.0, value * 1.15 + 0.25);
}

double PenaltySystem::weight(size_t a, size_t b) const {
    const auto it = weights_.find(key(a, b));
    return it == weights_.end() ? 1.0 : it->second;
}

} // namespace nest
