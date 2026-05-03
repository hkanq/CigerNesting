#pragma once

#include "core/mat2.h"
#include "core/vec2.h"

namespace nest {

struct Transform {
    Mat2 linear{};
    Vec2 translation{};

    constexpr Transform() = default;
    constexpr Transform(const Mat2& matrix, const Vec2& offset) : linear(matrix), translation(offset) {}

    static constexpr Transform identity() { return {}; }
    static constexpr Transform translated(double x, double y) { return {Mat2::identity(), {x, y}}; }
    static constexpr Transform scaled(double sx, double sy) { return {Mat2::scale(sx, sy), {0.0, 0.0}}; }
    static Transform rotated(double radians) { return {Mat2::rotation(radians), {0.0, 0.0}}; }

    constexpr Vec2 apply(const Vec2& p) const {
        return linear.apply(p) + translation;
    }

    constexpr Transform operator*(const Transform& rhs) const {
        return {linear * rhs.linear, linear.apply(rhs.translation) + translation};
    }
};

} // namespace nest
