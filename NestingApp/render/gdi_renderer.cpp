#include "render/gdi_renderer.h"

#include <algorithm>
#include <cmath>

namespace nest {

GdiRenderer::GdiRenderer(HDC hdc) : hdc_(hdc) {}

COLORREF GdiRenderer::toColorRef(Color color) {
    return RGB(color.r, color.g, color.b);
}

HPEN GdiRenderer::makePen(Color color, double thickness) const {
    return CreatePen(PS_SOLID, std::max(1, static_cast<int>(thickness)), toColorRef(color));
}

HBRUSH GdiRenderer::makeBrush(Color color) const {
    return CreateSolidBrush(toColorRef(color));
}

POINT GdiRenderer::toPoint(Vec2 p) const {
    return {static_cast<LONG>(std::lround(p.x)), static_cast<LONG>(std::lround(p.y))};
}

void GdiRenderer::beginFrame(int width, int height, Color clearColor) {
    RECT rect{0, 0, width, height};
    HBRUSH brush = makeBrush(clearColor);
    FillRect(hdc_, &rect, brush);
    DeleteObject(brush);
    SetBkMode(hdc_, TRANSPARENT);
}

void GdiRenderer::endFrame() {}

void GdiRenderer::drawLine(Vec2 a, Vec2 b, Color color, double thickness) {
    HPEN pen = makePen(color, thickness);
    HGDIOBJ oldPen = SelectObject(hdc_, pen);
    MoveToEx(hdc_, static_cast<int>(std::lround(a.x)), static_cast<int>(std::lround(a.y)), nullptr);
    LineTo(hdc_, static_cast<int>(std::lround(b.x)), static_cast<int>(std::lround(b.y)));
    SelectObject(hdc_, oldPen);
    DeleteObject(pen);
}

void GdiRenderer::drawPolyline(const std::vector<Vec2>& points, Color color, double thickness) {
    if (points.size() < 2) {
        return;
    }
    std::vector<POINT> native;
    native.reserve(points.size());
    for (const auto& point : points) {
        native.push_back(toPoint(point));
    }
    HPEN pen = makePen(color, thickness);
    HGDIOBJ oldPen = SelectObject(hdc_, pen);
    Polyline(hdc_, native.data(), static_cast<int>(native.size()));
    SelectObject(hdc_, oldPen);
    DeleteObject(pen);
}

void GdiRenderer::drawPolygon(const std::vector<Vec2>& points, Color fill, Color stroke, double thickness) {
    if (points.size() < 2) {
        return;
    }
    std::vector<POINT> native;
    native.reserve(points.size());
    for (const auto& point : points) {
        native.push_back(toPoint(point));
    }

    HPEN pen = makePen(stroke, thickness);
    HBRUSH brush = fill.a == 0 ? static_cast<HBRUSH>(GetStockObject(NULL_BRUSH)) : makeBrush(fill);
    HGDIOBJ oldPen = SelectObject(hdc_, pen);
    HGDIOBJ oldBrush = SelectObject(hdc_, brush);

    if (points.size() >= 3) {
        Polygon(hdc_, native.data(), static_cast<int>(native.size()));
    } else {
        Polyline(hdc_, native.data(), static_cast<int>(native.size()));
    }

    SelectObject(hdc_, oldBrush);
    SelectObject(hdc_, oldPen);
    if (fill.a != 0) {
        DeleteObject(brush);
    }
    DeleteObject(pen);
}

void GdiRenderer::drawText(Vec2 p, const std::wstring& text, Color color) {
    SetTextColor(hdc_, toColorRef(color));
    TextOutW(hdc_, static_cast<int>(std::lround(p.x)), static_cast<int>(std::lround(p.y)), text.c_str(), static_cast<int>(text.size()));
}

void GdiRenderer::fillRect(const RectD& rect, Color color) {
    RECT native{
        static_cast<LONG>(std::lround(rect.x)),
        static_cast<LONG>(std::lround(rect.y)),
        static_cast<LONG>(std::lround(rect.x + rect.width)),
        static_cast<LONG>(std::lround(rect.y + rect.height))
    };
    HBRUSH brush = makeBrush(color);
    FillRect(hdc_, &native, brush);
    DeleteObject(brush);
}

void GdiRenderer::strokeRect(const RectD& rect, Color color, double thickness) {
    HPEN pen = makePen(color, thickness);
    HGDIOBJ oldPen = SelectObject(hdc_, pen);
    HGDIOBJ oldBrush = SelectObject(hdc_, GetStockObject(NULL_BRUSH));
    Rectangle(hdc_, static_cast<int>(std::lround(rect.x)), static_cast<int>(std::lround(rect.y)), static_cast<int>(std::lround(rect.x + rect.width)), static_cast<int>(std::lround(rect.y + rect.height)));
    SelectObject(hdc_, oldBrush);
    SelectObject(hdc_, oldPen);
    DeleteObject(pen);
}

} // namespace nest
