#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include <vector>

namespace nest {

class LocalSearch {
public:
    void resolveSimpleCollisions(const Document& document, const EngineSettings& settings, std::vector<Pose>& poses) const;
};

} // namespace nest
