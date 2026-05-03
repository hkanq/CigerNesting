#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <string>

namespace nest {

class Toolbar {
public:
    bool create(HWND parent, HINSTANCE instance);
    void move(const RECT& rect);
    void setPhaseText(const std::wstring& text);
    HWND hwnd() const { return hwnd_; }

    static LRESULT CALLBACK wndProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam);

private:
    void createControls(HINSTANCE instance);
    void layoutControls();
    void paint();

    HWND hwnd_ = nullptr;
    HWND owner_ = nullptr;
    HWND openButton_ = nullptr;
    HWND saveButton_ = nullptr;
    HWND startButton_ = nullptr;
    HWND stopButton_ = nullptr;
    HWND corelButton_ = nullptr;
    HWND phaseLabel_ = nullptr;
};

} // namespace nest
