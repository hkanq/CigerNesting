#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "render/renderer.h"

namespace nest {

class GdiRenderer final : public IRenderer {
public:
    explicit GdiRenderer(HDC hdc);

    void beginFrame(int width, int height, Color clearColor) override;
    void endFrame() override;
    void drawLine(Vec2 a, Vec2 b, Color color, double thickness = 1.0) override;
    void drawPolyline(const std::vector<Vec2>& points, Color color, double thickness = 1.0) override;
    void drawPolygon(const std::vector<Vec2>& points, Color fill, Color stroke, double thickness = 1.0) override;
    void drawText(Vec2 p, const std::wstring& text, Color color) override;
    void fillRect(const RectD& rect, Color color) override;
    void strokeRect(const RectD& rect, Color color, double thickness = 1.0) override;

private:
    static COLORREF toColorRef(Color color);
    HPEN makePen(Color color, double thickness) const;
    HBRUSH makeBrush(Color color) const;
    POINT toPoint(Vec2 p) const;

    HDC hdc_ = nullptr;
};

} // namespace nest
