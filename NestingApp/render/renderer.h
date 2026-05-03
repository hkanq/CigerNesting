#pragma once

#include "core/vec2.h"
#include <cstdint>
#include <string>
#include <vector>

namespace nest {

struct Color {
    unsigned char r = 0;
    unsigned char g = 0;
    unsigned char b = 0;
    unsigned char a = 255;
};

struct RectD {
    double x = 0.0;
    double y = 0.0;
    double width = 0.0;
    double height = 0.0;
};

class IRenderer {
public:
    virtual ~IRenderer() = default;

    virtual void beginFrame(int width, int height, Color clearColor) = 0;
    virtual void endFrame() = 0;
    virtual void drawLine(Vec2 a, Vec2 b, Color color, double thickness = 1.0) = 0;
    virtual void drawPolyline(const std::vector<Vec2>& points, Color color, double thickness = 1.0) = 0;
    virtual void drawPolygon(const std::vector<Vec2>& points, Color fill, Color stroke, double thickness = 1.0) = 0;
    virtual void drawText(Vec2 p, const std::wstring& text, Color color) = 0;
    virtual void fillRect(const RectD& rect, Color color) = 0;
    virtual void strokeRect(const RectD& rect, Color color, double thickness = 1.0) = 0;
};

} // namespace nest
