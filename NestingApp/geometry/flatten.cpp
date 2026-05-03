#include "geometry/flatten.h"

#include "geometry/bezier.h"
#include "geometry/winding.h"

#include <utility>

namespace nest {

std::vector<Ring> flattenPathToRings(const Path& path, double tolerance) {
    std::vector<Ring> rings;
    Ring currentRing;
    Vec2 current{};
    Vec2 subpathStart{};
    bool hasCurrent = false;

    auto finishRing = [&]() {
        if (currentRing.points.size() >= 2) {
            currentRing.winding = windingDirection(currentRing.points);
            rings.push_back(std::move(currentRing));
        }
        currentRing = Ring{};
        hasCurrent = false;
    };

    for (const auto& command : path.commands()) {
        switch (command.type) {
        case PathCommandType::MoveTo:
            finishRing();
            current = command.p0;
            subpathStart = current;
            currentRing.points.push_back(current);
            hasCurrent = true;
            break;
        case PathCommandType::LineTo:
            if (!hasCurrent) {
                currentRing.points.push_back(command.p0);
                subpathStart = command.p0;
                hasCurrent = true;
            } else {
                current = command.p0;
                currentRing.points.push_back(current);
            }
            break;
        case PathCommandType::QuadraticTo:
            if (hasCurrent) {
                flattenQuadraticBezier(current, command.p0, command.p1, tolerance, currentRing.points);
                current = command.p1;
            }
            break;
        case PathCommandType::CubicTo:
            if (hasCurrent) {
                flattenCubicBezier(current, command.p0, command.p1, command.p2, tolerance, currentRing.points);
                current = command.p2;
            }
            break;
        case PathCommandType::ArcTo:
            break;
        case PathCommandType::Close:
            if (hasCurrent && !currentRing.points.empty() && !almostEqual(currentRing.points.back(), subpathStart)) {
                currentRing.points.push_back(subpathStart);
            }
            current = subpathStart;
            finishRing();
            break;
        }
    }

    finishRing();
    return rings;
}

} // namespace nest
