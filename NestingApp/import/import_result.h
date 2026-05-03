#pragma once

#include "core/part.h"
#include <string>
#include <vector>

namespace nest {

struct ImportResult {
    bool ok = false;
    std::wstring message;
    std::vector<Part> parts;
};

} // namespace nest
