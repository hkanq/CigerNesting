#include "engine/tabu_memory.h"

#include <cmath>
#include <cstdint>

namespace nest {
namespace {

size_t combineHash(size_t seed, size_t value) {
    return seed ^ (value + 0x9e3779b97f4a7c15ull + (seed << 6) + (seed >> 2));
}

int quantize(double value, double scale) {
    return static_cast<int>(std::llround(value / scale));
}

} // namespace

TabuMemory::TabuMemory(size_t capacity) : capacity_(capacity) {
}

size_t TabuMemory::poseHash(const Pose& pose) {
    size_t hash = 1469598103934665603ull;
    hash = combineHash(hash, static_cast<size_t>(quantize(pose.x, 1.0)));
    hash = combineHash(hash, static_cast<size_t>(quantize(pose.y, 1.0)));
    hash = combineHash(hash, static_cast<size_t>(quantize(pose.angleRadians, 0.0017453292519943296)));
    hash = combineHash(hash, pose.mirrored ? 1u : 0u);
    return hash;
}

size_t TabuMemory::layoutHash(const std::vector<Pose>& poses) {
    size_t hash = 1469598103934665603ull;
    for (const Pose& pose : poses) {
        hash = combineHash(hash, poseHash(pose));
    }
    return hash;
}

void TabuMemory::remember(size_t key, std::deque<size_t>& order, std::unordered_set<size_t>& set) {
    if (capacity_ == 0 || set.find(key) != set.end()) {
        return;
    }
    order.push_back(key);
    set.insert(key);
    while (order.size() > capacity_) {
        set.erase(order.front());
        order.pop_front();
    }
}

bool TabuMemory::containsMove(size_t partIndex, const Pose& from, const Pose& to) const {
    size_t key = combineHash(partIndex, poseHash(from));
    key = combineHash(key, poseHash(to));
    return moves_.find(key) != moves_.end();
}

void TabuMemory::rememberMove(size_t partIndex, const Pose& from, const Pose& to) {
    size_t key = combineHash(partIndex, poseHash(from));
    key = combineHash(key, poseHash(to));
    remember(key, moveOrder_, moves_);
}

bool TabuMemory::containsLayout(const std::vector<Pose>& poses) const {
    return layouts_.find(layoutHash(poses)) != layouts_.end();
}

void TabuMemory::rememberLayout(const std::vector<Pose>& poses) {
    remember(layoutHash(poses), layoutOrder_, layouts_);
}

} // namespace nest
