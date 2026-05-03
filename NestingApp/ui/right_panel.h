#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "engine/engine_settings.h"

namespace nest {

class RightPanel {
public:
    bool create(HWND parent, HINSTANCE instance);
    void move(const RECT& rect);
    EngineSettings getSettings() const;
    HWND hwnd() const { return hwnd_; }

    static LRESULT CALLBACK wndProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam);

private:
    void createControls(HINSTANCE instance);
    void layoutControls();
    void paint();
    double readDouble(HWND edit, double fallback) const;
    int readInt(HWND edit, int fallback) const;

    HWND hwnd_ = nullptr;
    HWND owner_ = nullptr;
    HWND sheetWidthEdit_ = nullptr;
    HWND sheetHeightEdit_ = nullptr;
    HWND spacingEdit_ = nullptr;
    HWND marginEdit_ = nullptr;
    HWND placementCombo_ = nullptr;
    HWND rotationCheck_ = nullptr;
    HWND rotationModeCombo_ = nullptr;
    HWND angleStepEdit_ = nullptr;
    HWND mirrorCheck_ = nullptr;
    HWND qualityCombo_ = nullptr;
    HWND timeLimitEdit_ = nullptr;
    HWND threadEdit_ = nullptr;
    HWND startButton_ = nullptr;
    HWND stopButton_ = nullptr;
    HWND corelExportButton_ = nullptr;
};

} // namespace nest
