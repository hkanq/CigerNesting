#pragma once

#include "core/polygon.h"
#include <cstddef>
#include <deque>
#include <unordered_set>
#include <vector>

namespace nest {

class TabuMemory {
public:
    explicit TabuMemory(size_t capacity = 256);

    bool containsMove(size_t partIndex, const Pose& from, const Pose& to) const;
    void rememberMove(size_t partIndex, const Pose& from, const Pose& to);

    bool containsLayout(const std::vector<Pose>& poses) const;
    void rememberLayout(const std::vector<Pose>& poses);

private:
    static size_t poseHash(const Pose& pose);
    static size_t layoutHash(const std::vector<Pose>& poses);
    void remember(size_t key, std::deque<size_t>& order, std::unordered_set<size_t>& set);

    size_t capacity_ = 256;
    std::deque<size_t> moveOrder_;
    std::deque<size_t> layoutOrder_;
    std::unordered_set<size_t> moves_;
    std::unordered_set<size_t> layouts_;
};

} // namespace nest
