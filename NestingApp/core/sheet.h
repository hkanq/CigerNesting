#pragma once

#include "core/polygon.h"
#include <vector>

namespace nest {

struct SheetProfile {
    Ring outerContour;
    std::vector<Ring> holes;
    std::vector<Ring> forbiddenZones;
    bool hasCustomProfile = false;
};

struct Sheet {
    double width = 1000.0;
    double height = 600.0;
    double margin = 10.0;
    Vec2 origin{0.0, 0.0};

    bool hasCustomProfile() const {
        return profile_.hasCustomProfile;
    }

    const SheetProfile& profile() const {
        return profile_;
    }

    void setProfile(const SheetProfile& profile) {
        profile_ = profile;
    }

    Ring makeRectangularOuterContour() const {
        Ring ring;
        ring.points = {
            origin,
            {origin.x + width, origin.y},
            {origin.x + width, origin.y + height},
            {origin.x, origin.y + height},
            origin
        };
        ring.isHole = false;
        ring.winding = 1;
        return ring;
    }

    void addUserPlacementPoint(Vec2 point) {
        userPlacementPoints_.push_back(point);
    }

    void clearUserPlacementPoints() {
        userPlacementPoints_.clear();
    }

    const std::vector<Vec2>& getUserPlacementPoints() const {
        return userPlacementPoints_;
    }

private:
    SheetProfile profile_{};
    std::vector<Vec2> userPlacementPoints_;
};

} // namespace nest
