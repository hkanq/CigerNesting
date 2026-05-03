#pragma once

#include "core/aabb.h"
#include "core/part.h"
#include "core/sheet.h"
#include <string>
#include <utility>
#include <vector>

namespace nest {

class Document {
public:
    Sheet sheet;
    std::wstring sourcePath;
    std::vector<Part> parts;

    void clear() {
        sourcePath.clear();
        parts.clear();
        sheet.clearUserPlacementPoints();
    }

    void addPart(Part part) {
        part.updateDerivedGeometry();
        parts.push_back(std::move(part));
    }

    AABB contentBounds() const {
        AABB box;
        box.include(AABB::fromMinMax(sheet.origin, {sheet.origin.x + sheet.width, sheet.origin.y + sheet.height}));
        for (const auto& part : parts) {
            box.include(transformedBounds(part, part.pose));
        }
        return box;
    }

    double totalPartArea() const {
        double total = 0.0;
        for (const auto& part : parts) {
            total += part.area;
        }
        return total;
    }
};

} // namespace nest
