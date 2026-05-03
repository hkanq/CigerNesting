#include "engine/compression.h"

#include "geometry/collision.h"
#include <algorithm>
#include <cmath>

namespace nest {
namespace {

bool candidateIsValid(const Document& document, const std::vector<Pose>& poses, size_t movingIndex, const Pose& candidate, const EngineSettings& settings) {
    if (!isPartInsideSheet(document.parts[movingIndex], candidate, document.sheet) ||
        overlapsSheetHolesOrForbiddenZones(document.parts[movingIndex], candidate, document.sheet)) {
        return false;
    }
    for (size_t i = 0; i < document.parts.size(); ++i) {
        if (i == movingIndex || i >= poses.size()) {
            continue;
        }
        if (partsOverlap(document.parts[movingIndex], candidate, document.parts[i], poses[i], settings.collisionTolerance)) {
            return false;
        }
        if (transformedBounds(document.parts[movingIndex], candidate).expanded(settings.partSpacing).overlaps(transformedBounds(document.parts[i], poses[i]))) {
            return false;
        }
    }
    return true;
}

int horizontalCompressionSign(PlacementStrategy strategy) {
    switch (strategy) {
    case PlacementStrategy::BottomRight:
    case PlacementStrategy::TopRight:
    case PlacementStrategy::RightToLeft:
        return 1;
    default:
        return -1;
    }
}

int verticalCompressionSign(PlacementStrategy strategy) {
    switch (strategy) {
    case PlacementStrategy::TopLeft:
    case PlacementStrategy::TopRight:
    case PlacementStrategy::TopToBottom:
        return 1;
    default:
        return -1;
    }
}

Vec2 nearestUserAnchor(const Document& document, const Vec2& center) {
    const auto& anchors = document.sheet.getUserPlacementPoints();
    Vec2 best = anchors.empty() ? Vec2{} : anchors.front();
    double bestDistance = anchors.empty() ? 0.0 : distance(center, best);
    for (const Vec2& anchor : anchors) {
        const double d = distance(center, anchor);
        if (d < bestDistance) {
            bestDistance = d;
            best = anchor;
        }
    }
    return best;
}

int signToward(double from, double to) {
    if (std::abs(to - from) < 1e-6) {
        return 0;
    }
    return to > from ? 1 : -1;
}

} // namespace

void Compression::compressLeftUp(const Document& document, const EngineSettings& settings, std::vector<Pose>& poses) const {
    const double step = std::max(1.0, settings.partSpacing * 0.5);
    for (int pass = 0; pass < 4; ++pass) {
        for (size_t i = 0; i < poses.size() && i < document.parts.size(); ++i) {
            for (int axis = 0; axis < 2; ++axis) {
                bool moved = true;
                while (moved) {
                    moved = false;
                    Pose candidate = poses[i];
                    if (axis == 0) {
                        candidate.x -= step;
                    } else {
                        candidate.y -= step;
                    }
                    if (candidateIsValid(document, poses, i, candidate, settings)) {
                        poses[i] = candidate;
                        moved = true;
                    }
                }
            }
        }
    }
}

void Compression::compressByStrategy(const Document& document, const EngineSettings& settings, std::vector<Pose>& poses) const {
    if (settings.placementStrategy == PlacementStrategy::BottomLeft ||
        settings.placementStrategy == PlacementStrategy::LeftToRight ||
        settings.placementStrategy == PlacementStrategy::BottomToTop) {
        compressLeftUp(document, settings, poses);
        return;
    }

    const double step = std::max(1.0, settings.partSpacing * 0.5);
    const Vec2 sheetCenter{
        document.sheet.origin.x + document.sheet.width * 0.5,
        document.sheet.origin.y + document.sheet.height * 0.5
    };

    for (int pass = 0; pass < 4; ++pass) {
        for (size_t i = 0; i < poses.size() && i < document.parts.size(); ++i) {
            for (int axis = 0; axis < 2; ++axis) {
                bool moved = true;
                const size_t maxMoves = static_cast<size_t>(std::max(1.0, (document.sheet.width + document.sheet.height) / step)) + 4;
                size_t moveCount = 0;
                while (moved && moveCount < maxMoves) {
                    moved = false;
                    Pose candidate = poses[i];
                    int sign = axis == 0
                        ? horizontalCompressionSign(settings.placementStrategy)
                        : verticalCompressionSign(settings.placementStrategy);
                    bool targetDriven = false;
                    double currentCoordinate = 0.0;
                    double targetCoordinate = 0.0;

                    if (settings.placementStrategy == PlacementStrategy::CenterOut ||
                        settings.placementStrategy == PlacementStrategy::OutsideIn ||
                        settings.placementStrategy == PlacementStrategy::UserPoints) {
                        const AABB box = transformedBounds(document.parts[i], poses[i]);
                        Vec2 target = sheetCenter;
                        if (settings.placementStrategy == PlacementStrategy::UserPoints && !document.sheet.getUserPlacementPoints().empty()) {
                            target = nearestUserAnchor(document, box.center());
                        }
                        currentCoordinate = axis == 0 ? box.center().x : box.center().y;
                        targetCoordinate = axis == 0 ? target.x : target.y;
                        sign = signToward(currentCoordinate, targetCoordinate);
                        targetDriven = true;
                    }

                    if (sign == 0) {
                        break;
                    }
                    if (targetDriven) {
                        const double currentDistance = std::abs(targetCoordinate - currentCoordinate);
                        const double nextDistance = std::abs(targetCoordinate - (currentCoordinate + static_cast<double>(sign) * step));
                        if (currentDistance <= step * 0.5 || nextDistance >= currentDistance - 1e-9) {
                            break;
                        }
                    }
                    if (axis == 0) {
                        candidate.x += static_cast<double>(sign) * step;
                    } else {
                        candidate.y += static_cast<double>(sign) * step;
                    }
                    if (candidateIsValid(document, poses, i, candidate, settings)) {
                        poses[i] = candidate;
                        moved = true;
                        ++moveCount;
                    }
                }
            }
        }
    }
}

} // namespace nest
