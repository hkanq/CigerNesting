#pragma once

#include "core/vec2.h"

namespace nest {

struct Sheet {
    double width = 1000.0;
    double height = 600.0;
    double margin = 10.0;
    Vec2 origin{0.0, 0.0};
};

} // namespace nest
