#include "engine/compression.h"

#include "geometry/collision.h"
#include <algorithm>

namespace nest {
namespace {

bool insideSheet(const Part& part, const Pose& pose, const EngineSettings& settings) {
    const AABB box = transformedBounds(part, pose);
    return box.min.x >= settings.margin - 1e-6 && box.min.y >= settings.margin - 1e-6 &&
        box.max.x <= settings.sheetWidth - settings.margin + 1e-6 &&
        box.max.y <= settings.sheetHeight - settings.margin + 1e-6;
}

bool candidateIsValid(const Document& document, const std::vector<Pose>& poses, size_t movingIndex, const Pose& candidate, const EngineSettings& settings) {
    if (!insideSheet(document.parts[movingIndex], candidate, settings)) {
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

} // namespace nest
