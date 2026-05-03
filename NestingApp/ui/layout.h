#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

namespace nest {

struct AppLayout {
    RECT toolbar{};
    RECT leftPanel{};
    RECT canvas{};
    RECT rightPanel{};
};

AppLayout calculateLayout(int width, int height);
int rectWidth(const RECT& rect);
int rectHeight(const RECT& rect);

} // namespace nest
