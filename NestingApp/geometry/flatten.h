#pragma once

#include "core/path.h"
#include "core/polygon.h"
#include <vector>

namespace nest {

std::vector<Ring> flattenPathToRings(const Path& path, double tolerance);

} // namespace nest
