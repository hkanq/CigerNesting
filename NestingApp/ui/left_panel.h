#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <cstddef>
#include <string>

namespace nest {

class LeftPanel {
public:
    bool create(HWND parent, HINSTANCE instance);
    void move(const RECT& rect);
    void setFileInfo(const std::wstring& path, size_t partCount);
    void setStats(size_t collisionCount, double utilization);
    HWND hwnd() const { return hwnd_; }

    static LRESULT CALLBACK wndProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam);

private:
    void paint();

    HWND hwnd_ = nullptr;
    std::wstring path_;
    size_t partCount_ = 0;
    size_t collisionCount_ = 0;
    double utilization_ = 0.0;
};

} // namespace nest
