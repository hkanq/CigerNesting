#pragma once

#include "core/vec2.h"
#include <algorithm>
#include <limits>

namespace nest {

struct AABB {
    Vec2 min{std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    Vec2 max{-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()};

    bool isValid() const { return min.x <= max.x && min.y <= max.y; }
    double width() const { return isValid() ? max.x - min.x : 0.0; }
    double height() const { return isValid() ? max.y - min.y : 0.0; }
    double area() const { return width() * height(); }
    Vec2 center() const { return {(min.x + max.x) * 0.5, (min.y + max.y) * 0.5}; }

    void include(const Vec2& p) {
        min.x = std::min(min.x, p.x);
        min.y = std::min(min.y, p.y);
        max.x = std::max(max.x, p.x);
        max.y = std::max(max.y, p.y);
    }

    void include(const AABB& other) {
        if (!other.isValid()) {
            return;
        }
        include(other.min);
        include(other.max);
    }

    AABB expanded(double amount) const {
        if (!isValid()) {
            return {};
        }
        return {{min.x - amount, min.y - amount}, {max.x + amount, max.y + amount}};
    }

    AABB translated(const Vec2& offset) const {
        if (!isValid()) {
            return {};
        }
        return {min + offset, max + offset};
    }

    bool overlaps(const AABB& other, double tolerance = 0.0) const {
        if (!isValid() || !other.isValid()) {
            return false;
        }
        return min.x <= other.max.x + tolerance && max.x + tolerance >= other.min.x &&
               min.y <= other.max.y + tolerance && max.y + tolerance >= other.min.y;
    }

    static AABB fromMinMax(const Vec2& pmin, const Vec2& pmax) {
        return {pmin, pmax};
    }
};

} // namespace nest
