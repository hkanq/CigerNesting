#pragma once

#include "core/document.h"
#include "engine/engine_settings.h"
#include <vector>

namespace nest {

class Compression {
public:
    void compressLeftUp(const Document& document, const EngineSettings& settings, std::vector<Pose>& poses) const;
    void compressByStrategy(const Document& document, const EngineSettings& settings, std::vector<Pose>& poses) const;
};

} // namespace nest
