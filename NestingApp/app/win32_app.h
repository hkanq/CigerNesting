#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

namespace nest {

class Win32App {
public:
    int run(HINSTANCE instance, int showCommand);
};

} // namespace nest
