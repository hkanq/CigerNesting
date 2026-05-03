#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "core/document.h"
#include "engine/solver_state.h"

namespace nest {

class CanvasView {
public:
    bool create(HWND parent, HINSTANCE instance);
    void move(const RECT& rect);
    void setDocument(Document* document);
    void setSnapshot(const SolverSnapshot& snapshot);
    void clearSnapshot();
    void fitToDocument();
    HWND hwnd() const { return hwnd_; }

    static LRESULT CALLBACK wndProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam);

private:
    void paint();
    void drawGrid(class IRenderer& renderer, int width, int height);
    void drawDocument(class IRenderer& renderer);
    Vec2 worldToScreen(Vec2 world) const;
    Vec2 screenToWorld(Vec2 screen) const;
    void adjustZoom(double wheelDelta, int mouseX, int mouseY);

    HWND hwnd_ = nullptr;
    Document* document_ = nullptr;
    SolverSnapshot snapshot_;
    bool hasSnapshot_ = false;
    double zoom_ = 1.0;
    Vec2 pan_{30.0, 30.0};
    bool dragging_ = false;
    POINT lastMouse_{};
};

} // namespace nest
