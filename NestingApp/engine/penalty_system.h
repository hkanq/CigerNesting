#pragma once

#include <cstddef>
#include <map>
#include <utility>

namespace nest {

class PenaltySystem {
public:
    void reset();
    void observeCollision(size_t a, size_t b);
    double weight(size_t a, size_t b) const;

private:
    static std::pair<size_t, size_t> key(size_t a, size_t b);
    std::map<std::pair<size_t, size_t>, double> weights_;
};

} // namespace nest
