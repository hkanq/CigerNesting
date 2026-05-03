#include "geometry/spatial_grid.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <set>

namespace nest {

SpatialGrid::SpatialGrid(double cellSize) : cellSize_(std::max(1.0, cellSize)) {}

void SpatialGrid::clear() {
    cells_.clear();
}

size_t SpatialGrid::CellHasher::operator()(const CellKey& key) const {
    const size_t hx = std::hash<int>{}(key.x);
    const size_t hy = std::hash<int>{}(key.y);
    return hx ^ (hy + 0x9e3779b97f4a7c15ull + (hx << 6) + (hx >> 2));
}

void SpatialGrid::insert(size_t id, const AABB& bounds) {
    if (!bounds.isValid()) {
        return;
    }
    const int minX = static_cast<int>(std::floor(bounds.min.x / cellSize_));
    const int maxX = static_cast<int>(std::floor(bounds.max.x / cellSize_));
    const int minY = static_cast<int>(std::floor(bounds.min.y / cellSize_));
    const int maxY = static_cast<int>(std::floor(bounds.max.y / cellSize_));

    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            cells_[{x, y}].push_back(id);
        }
    }
}

std::vector<std::pair<size_t, size_t>> SpatialGrid::candidatePairs() const {
    std::set<std::pair<size_t, size_t>> uniquePairs;
    for (const auto& [key, ids] : cells_) {
        (void)key;
        for (size_t i = 0; i < ids.size(); ++i) {
            for (size_t j = i + 1; j < ids.size(); ++j) {
                const auto a = std::min(ids[i], ids[j]);
                const auto b = std::max(ids[i], ids[j]);
                uniquePairs.insert({a, b});
            }
        }
    }
    return {uniquePairs.begin(), uniquePairs.end()};
}

} // namespace nest
