#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

namespace nest {

class ProgressBar {
public:
    bool create(HWND parent, HINSTANCE instance);
    void move(const RECT& rect);
    void setValue(double value);
    HWND hwnd() const { return hwnd_; }

    static LRESULT CALLBACK wndProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam);

private:
    void paint();

    HWND hwnd_ = nullptr;
    double value_ = 0.0;
};

} // namespace nest
