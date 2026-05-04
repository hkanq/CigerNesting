#include "engine/penalty_system.h"

#include <algorithm>

namespace nest {

void PenaltySystem::reset() {
    weights_.clear();
    collisionStreak_.clear();
    stalledStreak_.clear();
}

std::pair<size_t, size_t> PenaltySystem::key(size_t a, size_t b) {
    return {std::min(a, b), std::max(a, b)};
}

void PenaltySystem::observeCollision(size_t a, size_t b) {
    const auto pairKey = key(a, b);
    auto& value = weights_[pairKey];
    auto& streak = collisionStreak_[pairKey];
    ++streak;
    value = value <= 0.0 ? 1.0 : value;
    value = std::min(64.0, value * 1.12 + 0.20 + static_cast<double>(streak) * 0.04);
}

void PenaltySystem::observeResolved(size_t a, size_t b) {
    const auto pairKey = key(a, b);
    collisionStreak_[pairKey] = 0;
    stalledStreak_[pairKey] = 0;
    auto it = weights_.find(pairKey);
    if (it != weights_.end()) {
        it->second = std::max(1.0, it->second * 0.985);
    }
}

void PenaltySystem::observeStalledPair(size_t a, size_t b) {
    const auto pairKey = key(a, b);
    auto& stalled = stalledStreak_[pairKey];
    ++stalled;
    auto& value = weights_[pairKey];
    value = value <= 0.0 ? 1.0 : value;
    value = std::min(96.0, value + 0.35 + static_cast<double>(stalled) * 0.10);
}

double PenaltySystem::weight(size_t a, size_t b) const {
    const auto it = weights_.find(key(a, b));
    return it == weights_.end() ? 1.0 : it->second;
}

double PenaltySystem::diversificationBias(size_t a, size_t b) const {
    const auto it = stalledStreak_.find(key(a, b));
    if (it == stalledStreak_.end()) {
        return 1.0;
    }
    return 1.0 + std::min(2.0, static_cast<double>(it->second) * 0.20);
}

bool PenaltySystem::shouldDiversify(size_t a, size_t b) const {
    const auto it = stalledStreak_.find(key(a, b));
    return it != stalledStreak_.end() && it->second >= 2;
}

} // namespace nest
