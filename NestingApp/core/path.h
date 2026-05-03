#pragma once

#include "core/vec2.h"
#include <vector>

namespace nest {

enum class PathCommandType {
    MoveTo,
    LineTo,
    QuadraticTo,
    CubicTo,
    ArcTo,
    Close
};

struct PathCommand {
    PathCommandType type = PathCommandType::MoveTo;
    Vec2 p0{};
    Vec2 p1{};
    Vec2 p2{};
};

class Path {
public:
    void moveTo(Vec2 p) { commands_.push_back({PathCommandType::MoveTo, p, {}, {}}); }
    void lineTo(Vec2 p) { commands_.push_back({PathCommandType::LineTo, p, {}, {}}); }
    void quadraticTo(Vec2 control, Vec2 end) { commands_.push_back({PathCommandType::QuadraticTo, control, end, {}}); }
    void cubicTo(Vec2 c1, Vec2 c2, Vec2 end) { commands_.push_back({PathCommandType::CubicTo, c1, c2, end}); }
    void close() { commands_.push_back({PathCommandType::Close, {}, {}, {}}); }

    const std::vector<PathCommand>& commands() const { return commands_; }
    bool empty() const { return commands_.empty(); }
    void clear() { commands_.clear(); }

private:
    std::vector<PathCommand> commands_;
};

} // namespace nest
