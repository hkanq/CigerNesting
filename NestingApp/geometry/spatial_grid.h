#pragma once

#include "core/aabb.h"
#include <cstddef>
#include <unordered_map>
#include <utility>
#include <vector>

namespace nest {

class SpatialGrid {
public:
    explicit SpatialGrid(double cellSize = 64.0);

    void clear();
    void insert(size_t id, const AABB& bounds);
    std::vector<std::pair<size_t, size_t>> candidatePairs() const;

private:
    struct CellKey {
        int x = 0;
        int y = 0;
        bool operator==(const CellKey& other) const { return x == other.x && y == other.y; }
    };

    struct CellHasher {
        size_t operator()(const CellKey& key) const;
    };

    double cellSize_ = 64.0;
    std::unordered_map<CellKey, std::vector<size_t>, CellHasher> cells_;
};

} // namespace nest
