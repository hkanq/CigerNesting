#include "ui/progress_bar.h"

#include <algorithm>

namespace nest {
namespace {

const wchar_t* kProgressClassName = L"CigerNestingProgressBar";

void registerProgressClass(HINSTANCE instance) {
    static bool registered = false;
    if (registered) {
        return;
    }
    WNDCLASSW wc{};
    wc.hInstance = instance;
    wc.lpszClassName = kProgressClassName;
    wc.lpfnWndProc = ProgressBar::wndProc;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = nullptr;
    RegisterClassW(&wc);
    registered = true;
}

} // namespace

bool ProgressBar::create(HWND parent, HINSTANCE instance) {
    registerProgressClass(instance);
    hwnd_ = CreateWindowExW(0, kProgressClassName, L"", WS_CHILD | WS_VISIBLE, 0, 0, 100, 20, parent, nullptr, instance, this);
    return hwnd_ != nullptr;
}

void ProgressBar::move(const RECT& rect) {
    if (hwnd_) {
        MoveWindow(hwnd_, rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top, TRUE);
    }
}

void ProgressBar::setValue(double value) {
    value_ = std::max(0.0, std::min(1.0, value));
    if (hwnd_) {
        InvalidateRect(hwnd_, nullptr, FALSE);
    }
}

LRESULT CALLBACK ProgressBar::wndProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam) {
    if (message == WM_NCCREATE) {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        auto* self = static_cast<ProgressBar*>(create->lpCreateParams);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(self));
        return TRUE;
    }

    auto* self = reinterpret_cast<ProgressBar*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    if (!self) {
        return DefWindowProcW(hwnd, message, wparam, lparam);
    }

    switch (message) {
    case WM_PAINT:
        self->paint();
        return 0;
    default:
        return DefWindowProcW(hwnd, message, wparam, lparam);
    }
}

void ProgressBar::paint() {
    PAINTSTRUCT ps{};
    HDC hdc = BeginPaint(hwnd_, &ps);
    RECT rc{};
    GetClientRect(hwnd_, &rc);

    HBRUSH bg = CreateSolidBrush(RGB(238, 241, 245));
    FillRect(hdc, &rc, bg);
    DeleteObject(bg);

    RECT fill = rc;
    fill.right = fill.left + static_cast<LONG>((fill.right - fill.left) * value_);
    HBRUSH fg = CreateSolidBrush(RGB(48, 124, 112));
    FillRect(hdc, &fill, fg);
    DeleteObject(fg);

    HPEN border = CreatePen(PS_SOLID, 1, RGB(170, 180, 190));
    HGDIOBJ oldPen = SelectObject(hdc, border);
    HGDIOBJ oldBrush = SelectObject(hdc, GetStockObject(NULL_BRUSH));
    Rectangle(hdc, rc.left, rc.top, rc.right, rc.bottom);
    SelectObject(hdc, oldBrush);
    SelectObject(hdc, oldPen);
    DeleteObject(border);

    EndPaint(hwnd_, &ps);
}

} // namespace nest
