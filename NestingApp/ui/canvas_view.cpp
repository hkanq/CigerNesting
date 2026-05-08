#include "ui/canvas_view.h"

#include "core/math_utils.h"
#include "geometry/transformed_shape.h"
#include "localization/localization.h"
#include "render/gdi_renderer.h"
#include <windowsx.h>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

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
    case SolverPhase::ContactPacking: return TextId::ContactPacking;
    case SolverPhase::GapFilling: return TextId::GapFilling;
    case SolverPhase::Rearrangement: return TextId::Rearrangement;
    case SolverPhase::Escape: return TextId::Escape;
    case SolverPhase::UltraRefinement: return TextId::UltraRefinement;
    case SolverPhase::FinalValidation: return TextId::FinalValidation;
    case SolverPhase::Done: return TextId::Done;
    case SolverPhase::NoValidLayout: return TextId::NoValidLayout;
    case SolverPhase::Failed: return TextId::Failed;
    case SolverPhase::Stopped: return TextId::Stopped;
    }
    return TextId::Idle;
}

TextId textIdForStrategy(SolverStrategy strategy) {
    switch (strategy) {
    case SolverStrategy::Idle: return TextId::Idle;
    case SolverStrategy::AdaptiveSearch: return TextId::Exploration;
    case SolverStrategy::ContactPacking: return TextId::ContactPacking;
    case SolverStrategy::Compression: return TextId::Compression;
    case SolverStrategy::GapFilling: return TextId::GapFilling;
    case SolverStrategy::HoleFilling: return TextId::HoleFilling;
    case SolverStrategy::ConcavityFilling: return TextId::ConcavityFilling;
    case SolverStrategy::SmallPartFiller: return TextId::SmallPartFiller;
    case SolverStrategy::Swap: return TextId::Swap;
    case SolverStrategy::EjectionChain: return TextId::EjectionChain;
    case SolverStrategy::ClusterRepack: return TextId::ClusterRepack;
    case SolverStrategy::RegionRepack: return TextId::RegionRepack;
    case SolverStrategy::RotationRefinement: return TextId::UltraRefinement;
    case SolverStrategy::Mirror: return TextId::Mirror;
    case SolverStrategy::Escape: return TextId::Escape;
    case SolverStrategy::Frontier: return TextId::Frontier;
    case SolverStrategy::Done: return TextId::Done;
    }
    return TextId::Idle;
}

void appendMoveCount(std::wostringstream& out, bool& first, TextId label, size_t count) {
    if (count == 0) {
        return;
    }
    if (!first) {
        out << L", ";
    }
    first = false;
    out << Localization::instance().text(label) << L" " << count;
}

std::wstring activeMovesText(const SolverSnapshot& snapshot) {
    std::wostringstream out;
    const ActiveMoveSummary& moves = snapshot.activeMoves;
    bool first = true;
    appendMoveCount(out, first, TextId::ContactPacking, moves.contact);
    appendMoveCount(out, first, TextId::Compression, moves.compression);
    appendMoveCount(out, first, TextId::GapFilling, moves.gap);
    appendMoveCount(out, first, TextId::HoleFilling, moves.hole);
    appendMoveCount(out, first, TextId::ConcavityFilling, moves.concavity);
    appendMoveCount(out, first, TextId::SmallPartFiller, moves.smallPart);
    appendMoveCount(out, first, TextId::Swap, moves.swap);
    appendMoveCount(out, first, TextId::EjectionChain, moves.chain);
    appendMoveCount(out, first, TextId::ClusterRepack, moves.cluster);
    appendMoveCount(out, first, TextId::RegionRepack, moves.region);
    appendMoveCount(out, first, TextId::UltraRefinement, moves.rotation);
    appendMoveCount(out, first, TextId::Mirror, moves.mirror);
    appendMoveCount(out, first, TextId::Escape, moves.escape);
    appendMoveCount(out, first, TextId::Frontier, moves.frontier);
    if (first) {
        out << Localization::instance().text(TextId::ContactPacking) << L" 0, "
            << Localization::instance().text(TextId::Compression) << L" 0, "
            << Localization::instance().text(TextId::UltraRefinement) << L" 0, "
            << Localization::instance().text(TextId::HoleFilling) << L" 0, "
            << Localization::instance().text(TextId::Swap) << L" 0";
    }
    return out.str();
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
    if (hasSnapshot_ &&
        snapshot.versionId == snapshot_.versionId &&
        !snapshot.layoutChanged &&
        snapshot.phase == snapshot_.phase &&
        snapshot.currentStrategy == snapshot_.currentStrategy &&
        snapshot.collisionCount == snapshot_.collisionCount &&
        std::abs(snapshot.utilization - snapshot_.utilization) < 1e-9) {
        return;
    }
    if (hwnd_) {
        RECT dirty{};
        bool hasDirty = false;
        if (document_ && hasSnapshot_ && snapshot.layoutChanged && !snapshot.changedParts.empty()) {
            auto displayPoses = [&](const SolverSnapshot& value) -> const std::vector<Pose>* {
                if (value.layoutChanged && value.currentPoses.size() == document_->parts.size()) {
                    return &value.currentPoses;
                }
                if (value.bestPoses.size() == document_->parts.size()) {
                    return &value.bestPoses;
                }
                if (value.currentPoses.size() == document_->parts.size()) {
                    return &value.currentPoses;
                }
                return nullptr;
            };
            const std::vector<Pose>* oldPoses = displayPoses(snapshot_);
            const std::vector<Pose>* newPoses = displayPoses(snapshot);
            AABB dirtyBounds;
            if (oldPoses && newPoses) {
                for (size_t index : snapshot.changedParts) {
                    if (index >= document_->parts.size() || index >= oldPoses->size() || index >= newPoses->size()) {
                        continue;
                    }
                    dirtyBounds.include(transformedBounds(document_->parts[index], (*oldPoses)[index]));
                    dirtyBounds.include(transformedBounds(document_->parts[index], (*newPoses)[index]));
                }
            }
            if (dirtyBounds.isValid()) {
                const Vec2 a = worldToScreen(dirtyBounds.min);
                const Vec2 b = worldToScreen(dirtyBounds.max);
                dirty.left = static_cast<LONG>(std::floor(std::min(a.x, b.x))) - 16;
                dirty.top = static_cast<LONG>(std::floor(std::min(a.y, b.y))) - 16;
                dirty.right = static_cast<LONG>(std::ceil(std::max(a.x, b.x))) + 16;
                dirty.bottom = static_cast<LONG>(std::ceil(std::max(a.y, b.y))) + 16;
                hasDirty = true;
            }
        }
        snapshot_ = snapshot;
        hasSnapshot_ = !snapshot.currentPoses.empty() || !snapshot.bestPoses.empty();
        InvalidateRect(hwnd_, hasDirty ? &dirty : nullptr, FALSE);
        return;
    }
    snapshot_ = snapshot;
    hasSnapshot_ = !snapshot.currentPoses.empty() || !snapshot.bestPoses.empty();
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
    if (hasSnapshot_ && snapshot_.layoutChanged && snapshot_.currentPoses.size() == document_->parts.size()) {
        poses = &snapshot_.currentPoses;
    } else if (hasSnapshot_ && snapshot_.bestPoses.size() == document_->parts.size()) {
        poses = &snapshot_.bestPoses;
    } else if (hasSnapshot_ && snapshot_.currentPoses.size() == document_->parts.size()) {
        poses = &snapshot_.currentPoses;
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
            const bool subsetMoved = hasSnapshot_ && snapshot_.layoutChanged &&
                std::find(snapshot_.changedParts.begin(), snapshot_.changedParts.end(), i) != snapshot_.changedParts.end();
            const bool movedPart = hasSnapshot_ && snapshot_.layoutChanged && (snapshot_.lastMovedPart == i || subsetMoved);
            const Color stroke = movedPart ? Color{220, 74, 42, 255} : Color{32, 68, 74, 255};
            const double strokeWidth = movedPart ? 3.0 : 1.5;
            if (ring.points.size() >= 3 && ring.closed()) {
                renderer.drawPolygon(screenPoints, ring.isHole ? Color{255, 255, 255, 255} : partFill(i), stroke, strokeWidth);
            } else {
                renderer.drawPolyline(screenPoints, stroke, strokeWidth);
            }
        }
    }

    if (hasSnapshot_) {
        const auto& loc = Localization::instance();
        std::wostringstream overlay;
        const bool terminal = snapshot_.phase == SolverPhase::Done ||
            snapshot_.phase == SolverPhase::NoValidLayout ||
            snapshot_.phase == SolverPhase::Failed ||
            snapshot_.phase == SolverPhase::Stopped;
        overlay << (terminal ? loc.text(TextId::Phase) : loc.text(TextId::ActiveMoves))
                << L": " << (terminal ? loc.text(textIdForPhase(snapshot_.phase)) : activeMovesText(snapshot_))
                << L"  |  " << loc.text(TextId::Collision) << L": " << snapshot_.collisionCount
                << L"  |  " << loc.text(TextId::Utilization) << L": " << static_cast<int>(snapshot_.utilization * 100.0) << L"%";
        renderer.drawText({18, 18}, overlay.str(), {40, 52, 60, 255});
        if (snapshot_.lastMovedPart != kNoPartIndex && !terminal) {
            std::wostringstream debug;
            debug << loc.text(TextId::LastMove) << L": #" << snapshot_.lastMovedPart
                  << L" " << loc.text(textIdForStrategy(snapshot_.lastMoveStrategy));
            if (snapshot_.bestUpdated) {
                debug << L" | " << loc.text(TextId::BestUpdate);
            }
            if (snapshot_.rebuildAttempt > 0) {
                debug << L" | Rebuild " << snapshot_.rebuildAttempt
                      << L" depth " << snapshot_.beamDepth
                      << L"/" << snapshot_.subsetSize;
                if (snapshot_.previewTemporary) {
                    debug << L" preview";
                }
            }
            renderer.drawText({18, 42}, debug.str(), {148, 65, 35, 255});
        }
    }
}

} // namespace nest
