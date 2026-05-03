#include "ui/left_panel.h"

#include <sstream>

namespace nest {
namespace {

const wchar_t* kLeftPanelClassName = L"CigerNestingLeftPanel";

void registerLeftPanelClass(HINSTANCE instance) {
    static bool registered = false;
    if (registered) {
        return;
    }
    WNDCLASSW wc{};
    wc.hInstance = instance;
    wc.lpszClassName = kLeftPanelClassName;
    wc.lpfnWndProc = LeftPanel::wndProc;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = nullptr;
    RegisterClassW(&wc);
    registered = true;
}

std::wstring basenameOf(const std::wstring& path) {
    const size_t slash = path.find_last_of(L"\\/");
    return slash == std::wstring::npos ? path : path.substr(slash + 1);
}

} // namespace

bool LeftPanel::create(HWND parent, HINSTANCE instance) {
    registerLeftPanelClass(instance);
    hwnd_ = CreateWindowExW(0, kLeftPanelClassName, L"", WS_CHILD | WS_VISIBLE, 0, 0, 240, 400, parent, nullptr, instance, this);
    return hwnd_ != nullptr;
}

void LeftPanel::move(const RECT& rect) {
    if (hwnd_) {
        MoveWindow(hwnd_, rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top, TRUE);
    }
}

void LeftPanel::setFileInfo(const std::wstring& path, size_t partCount) {
    path_ = path.empty() ? L"No file loaded" : basenameOf(path);
    partCount_ = partCount;
    if (hwnd_) {
        InvalidateRect(hwnd_, nullptr, FALSE);
    }
}

void LeftPanel::setStats(size_t collisionCount, double utilization) {
    collisionCount_ = collisionCount;
    utilization_ = utilization;
    if (hwnd_) {
        InvalidateRect(hwnd_, nullptr, FALSE);
    }
}

LRESULT CALLBACK LeftPanel::wndProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam) {
    if (message == WM_NCCREATE) {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        auto* self = static_cast<LeftPanel*>(create->lpCreateParams);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(self));
        return TRUE;
    }

    auto* self = reinterpret_cast<LeftPanel*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    if (!self) {
        return DefWindowProcW(hwnd, message, wparam, lparam);
    }

    if (message == WM_PAINT) {
        self->paint();
        return 0;
    }
    return DefWindowProcW(hwnd, message, wparam, lparam);
}

void LeftPanel::paint() {
    PAINTSTRUCT ps{};
    HDC hdc = BeginPaint(hwnd_, &ps);
    RECT rc{};
    GetClientRect(hwnd_, &rc);
    HBRUSH bg = CreateSolidBrush(RGB(250, 251, 247));
    FillRect(hdc, &rc, bg);
    DeleteObject(bg);
    SetBkMode(hdc, TRANSPARENT);
    SetTextColor(hdc, RGB(35, 45, 55));

    RECT text{16, 18, rc.right - 12, rc.bottom - 12};
    DrawTextW(hdc, L"Dosya Paneli", -1, &text, DT_LEFT | DT_TOP | DT_SINGLELINE);
    text.top += 34;
    DrawTextW(hdc, path_.c_str(), -1, &text, DT_LEFT | DT_TOP | DT_WORDBREAK);

    std::wostringstream stats;
    stats << L"\nParca sayisi: " << partCount_ << L"\nCarpisma: " << collisionCount_ << L"\nDoluluk: " << static_cast<int>(utilization_ * 100.0) << L"%";
    text.top += 54;
    DrawTextW(hdc, stats.str().c_str(), -1, &text, DT_LEFT | DT_TOP | DT_WORDBREAK);

    RECT hint{16, rc.bottom - 86, rc.right - 14, rc.bottom - 12};
    SetTextColor(hdc, RGB(94, 105, 116));
    DrawTextW(hdc, L"SVG, PLT/HPGL veya DXF acin. Mouse tekerlegi zoom, sol surukleme pan.", -1, &hint, DT_LEFT | DT_TOP | DT_WORDBREAK);

    HPEN line = CreatePen(PS_SOLID, 1, RGB(218, 222, 210));
    HGDIOBJ oldPen = SelectObject(hdc, line);
    MoveToEx(hdc, rc.right - 1, 0, nullptr);
    LineTo(hdc, rc.right - 1, rc.bottom);
    SelectObject(hdc, oldPen);
    DeleteObject(line);
    EndPaint(hwnd_, &ps);
}

} // namespace nest
