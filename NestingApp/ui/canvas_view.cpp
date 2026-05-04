#include "ui/canvas_view.h"

#include "core/math_utils.h"
#include "localization/localization.h"
#include "render/gdi_renderer.h"
#include <windowsx.h>
#include <algorithm>
#include <cmath>
#include <sstream>

namespace nest {
namespace {

const wchar_t* kCanvasClassName = L"CigerNestingCanvasView";

void registerCanvasClass(HINSTANCE instance) {
    static bool registered = false;
    if (registered) {
        return;
    }
    WNDCLASSW wc{};
    wc.hInstance = instance;
    wc.lpszClassName = kCanvasClassName;
    wc.lpfnWndProc = CanvasView::wndProc;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = nullptr;
    RegisterClassW(&wc);
    registered = true;
}

Color partFill(size_t index) {
    const Color palette[] = {
        {96, 150, 144, 70}, {206, 132, 88, 70}, {88, 132, 188, 70},
        {176, 154, 78, 70}, {138, 120, 168, 70}, {90, 160, 104, 70}
    };
    return palette[index % (sizeof(palette) / sizeof(palette[0]))];
}

TextId textIdForPhase(SolverPhase phase) {
    switch (phase) {
    case SolverPhase::Idle: return TextId::Idle;
    case SolverPhase::PrepareGeometry: return TextId::PrepareGeometry;
    case SolverPhase::InitialPlacement: return TextId::InitialPlacement;
    case SolverPhase::Exploration: return TextId::Exploration;
    case SolverPhase::CollisionResolution: return TextId::CollisionResolution;
    case SolverPhase::Compression: return TextId::Compression;
    case SolverPhase::GapFilling: return TextId::GapFilling;
    case SolverPhase::Rearrangement: return TextId::Rearrangement;
    case SolverPhase::UltraRefinement: return TextId::UltraRefinement;
    case SolverPhase::FinalValidation: return TextId::FinalValidation;
    case SolverPhase::Done: return TextId::Done;
    case SolverPhase::Stopped: return TextId::Stopped;
    }
    return TextId::Idle;
}

} // namespace

bool CanvasView::create(HWND parent, HINSTANCE instance) {
    registerCanvasClass(instance);
    hwnd_ = CreateWindowExW(0, kCanvasClassName, L"", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS, 0, 0, 400, 300, parent, nullptr, instance, this);
    return hwnd_ != nullptr;
}

void CanvasView::move(const RECT& rect) {
    if (hwnd_) {
        MoveWindow(hwnd_, rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top, TRUE);
    }
}

void CanvasView::setDocument(Document* document) {
    document_ = document;
    if (hwnd_) {
        InvalidateRect(hwnd_, nullptr, FALSE);
    }
}

void CanvasView::setSnapshot(const SolverSnapshot& snapshot) {
    snapshot_ = snapshot;
    hasSnapshot_ = !snapshot.currentPoses.empty() || !snapshot.bestPoses.empty();
    if (hwnd_) {
        InvalidateRect(hwnd_, nullptr, FALSE);
    }
}

void CanvasView::clearSnapshot() {
    snapshot_ = {};
    hasSnapshot_ = false;
    if (hwnd_) {
        InvalidateRect(hwnd_, nullptr, FALSE);
    }
}

void CanvasView::fitToDocument() {
    if (!document_ || !hwnd_) {
        return;
    }
    RECT rc{};
    GetClientRect(hwnd_, &rc);
    const int width = std::max(1, static_cast<int>(rc.right - rc.left));
    const int height = std::max(1, static_cast<int>(rc.bottom - rc.top));
    AABB bounds = document_->contentBounds();
    if (!bounds.isValid() || bounds.width() <= 1e-6 || bounds.height() <= 1e-6) {
        return;
    }
    const double sx = (width - 50.0) / bounds.width();
    const double sy = (height - 50.0) / bounds.height();
    zoom_ = std::max(0.02, std::min(sx, sy));
    pan_ = {25.0 - bounds.min.x * zoom_, 25.0 - bounds.min.y * zoom_};
    InvalidateRect(hwnd_, nullptr, FALSE);
}

Vec2 CanvasView::worldToScreen(Vec2 world) const {
    return {world.x * zoom_ + pan_.x, world.y * zoom_ + pan_.y};
}

Vec2 CanvasView::screenToWorld(Vec2 screen) const {
    return {(screen.x - pan_.x) / zoom_, (screen.y - pan_.y) / zoom_};
}

void CanvasView::adjustZoom(double wheelDelta, int mouseX, int mouseY) {
    const Vec2 before = screenToWorld({static_cast<double>(mouseX), static_cast<double>(mouseY)});
    const double factor = wheelDelta > 0.0 ? 1.12 : 1.0 / 1.12;
    zoom_ = std::max(0.02, std::min(50.0, zoom_ * factor));
    const Vec2 afterScreen = worldToScreen(before);
    pan_.x += static_cast<double>(mouseX) - afterScreen.x;
    pan_.y += static_cast<double>(mouseY) - afterScreen.y;
}

LRESULT CALLBACK CanvasView::wndProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam) {
    if (message == WM_NCCREATE) {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        auto* self = static_cast<CanvasView*>(create->lpCreateParams);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(self));
        return TRUE;
    }

    auto* self = reinterpret_cast<CanvasView*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    if (!self) {
        return DefWindowProcW(hwnd, message, wparam, lparam);
    }

    switch (message) {
    case WM_PAINT:
        self->paint();
        return 0;
    case WM_MOUSEWHEEL: {
        POINT pt{GET_X_LPARAM(lparam), GET_Y_LPARAM(lparam)};
        ScreenToClient(hwnd, &pt);
        self->adjustZoom(static_cast<double>(GET_WHEEL_DELTA_WPARAM(wparam)), pt.x, pt.y);
        InvalidateRect(hwnd, nullptr, FALSE);
        return 0;
    }
    case WM_LBUTTONDOWN:
        self->dragging_ = true;
        self->lastMouse_ = {GET_X_LPARAM(lparam), GET_Y_LPARAM(lparam)};
        SetCapture(hwnd);
        return 0;
    case WM_LBUTTONUP:
        self->dragging_ = false;
        ReleaseCapture();
        return 0;
    case WM_MOUSEMOVE:
        if (self->dragging_) {
            POINT now{GET_X_LPARAM(lparam), GET_Y_LPARAM(lparam)};
            self->pan_.x += now.x - self->lastMouse_.x;
            self->pan_.y += now.y - self->lastMouse_.y;
            self->lastMouse_ = now;
            InvalidateRect(hwnd, nullptr, FALSE);
        }
        return 0;
    default:
        return DefWindowProcW(hwnd, message, wparam, lparam);
    }
}

void CanvasView::paint() {
    PAINTSTRUCT ps{};
    HDC hdc = BeginPaint(hwnd_, &ps);
    RECT rc{};
    GetClientRect(hwnd_, &rc);
    GdiRenderer renderer(hdc);
    renderer.beginFrame(rc.right - rc.left, rc.bottom - rc.top, {252, 252, 249, 255});
    drawGrid(renderer, rc.right - rc.left, rc.bottom - rc.top);
    drawDocument(renderer);
    renderer.endFrame();
    EndPaint(hwnd_, &ps);
}

void CanvasView::drawGrid(IRenderer& renderer, int width, int height) {
    const double grid = 50.0;
    const Vec2 worldMin = screenToWorld({0.0, 0.0});
    const Vec2 worldMax = screenToWorld({static_cast<double>(width), static_cast<double>(height)});
    const double startX = std::floor(worldMin.x / grid) * grid;
    const double startY = std::floor(worldMin.y / grid) * grid;

    for (double x = startX; x <= worldMax.x; x += grid) {
        renderer.drawLine(worldToScreen({x, worldMin.y}), worldToScreen({x, worldMax.y}), {230, 232, 226, 255}, 1.0);
    }
    for (double y = startY; y <= worldMax.y; y += grid) {
        renderer.drawLine(worldToScreen({worldMin.x, y}), worldToScreen({worldMax.x, y}), {230, 232, 226, 255}, 1.0);
    }
}

void CanvasView::drawDocument(IRenderer& renderer) {
    if (!document_) {
        renderer.drawText({24, 24}, Localization::instance().text(TextId::PreviewPrompt), {70, 82, 92, 255});
        return;
    }

    const Vec2 sheetA = worldToScreen(document_->sheet.origin);
    const Vec2 sheetB = worldToScreen({document_->sheet.origin.x + document_->sheet.width, document_->sheet.origin.y + document_->sheet.height});
    renderer.fillRect({sheetA.x, sheetA.y, sheetB.x - sheetA.x, sheetB.y - sheetA.y}, {255, 255, 255, 255});
    renderer.strokeRect({sheetA.x, sheetA.y, sheetB.x - sheetA.x, sheetB.y - sheetA.y}, {54, 66, 74, 255}, 2.0);

    const std::vector<Pose>* poses = nullptr;
    if (hasSnapshot_ && snapshot_.currentPoses.size() == document_->parts.size()) {
        poses = &snapshot_.currentPoses;
    } else if (hasSnapshot_ && snapshot_.bestPoses.size() == document_->parts.size()) {
        poses = &snapshot_.bestPoses;
    }

    for (size_t i = 0; i < document_->parts.size(); ++i) {
        const Part& part = document_->parts[i];
        const Pose pose = poses ? (*poses)[i] : part.pose;
        const Transform transform = pose.toTransform();
        for (const auto& ring : part.rings) {
            std::vector<Vec2> screenPoints;
            screenPoints.reserve(ring.points.size());
            for (const auto& point : ring.points) {
                screenPoints.push_back(worldToScreen(transform.apply(point)));
            }
            if (ring.points.size() >= 3 && ring.closed()) {
                renderer.drawPolygon(screenPoints, ring.isHole ? Color{255, 255, 255, 255} : partFill(i), {32, 68, 74, 255}, 1.5);
            } else {
                renderer.drawPolyline(screenPoints, {32, 68, 74, 255}, 1.5);
            }
        }
    }

    if (hasSnapshot_) {
        const auto& loc = Localization::instance();
        std::wostringstream overlay;
        overlay << loc.text(TextId::Phase) << L": " << loc.text(textIdForPhase(snapshot_.phase))
                << L"  |  " << loc.text(TextId::Collision) << L": " << snapshot_.collisionCount
                << L"  |  " << loc.text(TextId::Utilization) << L": " << static_cast<int>(snapshot_.utilization * 100.0) << L"%";
        renderer.drawText({18, 18}, overlay.str(), {40, 52, 60, 255});
    }
}

} // namespace nest
