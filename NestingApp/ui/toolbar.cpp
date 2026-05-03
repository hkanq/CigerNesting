#include "ui/toolbar.h"

#include "localization/localization.h"
#include "ui/control_ids.h"
#include <array>

namespace nest {
namespace {

const wchar_t* kToolbarClassName = L"CigerNestingToolbar";

void registerToolbarClass(HINSTANCE instance) {
    static bool registered = false;
    if (registered) {
        return;
    }
    WNDCLASSW wc{};
    wc.hInstance = instance;
    wc.lpszClassName = kToolbarClassName;
    wc.lpfnWndProc = Toolbar::wndProc;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = nullptr;
    RegisterClassW(&wc);
    registered = true;
}

} // namespace

bool Toolbar::create(HWND parent, HINSTANCE instance) {
    owner_ = parent;
    registerToolbarClass(instance);
    hwnd_ = CreateWindowExW(0, kToolbarClassName, L"", WS_CHILD | WS_VISIBLE | WS_CLIPCHILDREN, 0, 0, 100, 54, parent, nullptr, instance, this);
    if (!hwnd_) {
        return false;
    }
    createControls(instance);
    return true;
}

void Toolbar::createControls(HINSTANCE instance) {
    const auto& loc = Localization::instance();
    openButton_ = CreateWindowW(L"BUTTON", loc.text(TextId::FileOpen), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 90, 28, hwnd_, reinterpret_cast<HMENU>(uiid::buttonOpen), instance, nullptr);
    saveButton_ = CreateWindowW(L"BUTTON", loc.text(TextId::Save), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 70, 28, hwnd_, reinterpret_cast<HMENU>(uiid::buttonSave), instance, nullptr);
    startButton_ = CreateWindowW(L"BUTTON", loc.text(TextId::Start), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 70, 28, hwnd_, reinterpret_cast<HMENU>(uiid::buttonStart), instance, nullptr);
    stopButton_ = CreateWindowW(L"BUTTON", loc.text(TextId::Stop), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 70, 28, hwnd_, reinterpret_cast<HMENU>(uiid::buttonStop), instance, nullptr);
    corelButton_ = CreateWindowW(L"BUTTON", loc.text(TextId::CorelConnection), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 130, 28, hwnd_, reinterpret_cast<HMENU>(uiid::buttonCorelConnect), instance, nullptr);
    phaseLabel_ = CreateWindowW(L"STATIC", loc.text(TextId::Idle), WS_CHILD | WS_VISIBLE | SS_LEFT, 0, 0, 200, 22, hwnd_, nullptr, instance, nullptr);
    layoutControls();
}

void Toolbar::move(const RECT& rect) {
    if (hwnd_) {
        MoveWindow(hwnd_, rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top, TRUE);
        layoutControls();
    }
}

void Toolbar::setPhaseText(const std::wstring& text) {
    if (phaseLabel_) {
        SetWindowTextW(phaseLabel_, text.c_str());
    }
}

void Toolbar::layoutControls() {
    if (!hwnd_) {
        return;
    }
    RECT rc{};
    GetClientRect(hwnd_, &rc);
    int x = 12;
    const int y = 13;
    const std::array<HWND, 5> buttons{openButton_, saveButton_, startButton_, stopButton_, corelButton_};
    const std::array<int, 5> widths{104, 82, 82, 82, 158};
    for (size_t i = 0; i < buttons.size(); ++i) {
        if (buttons[i]) {
            MoveWindow(buttons[i], x, y, widths[i], 28, TRUE);
            x += widths[i] + 8;
        }
    }
    if (phaseLabel_) {
        MoveWindow(phaseLabel_, x + 12, y + 5, 220, 22, TRUE);
    }
}

LRESULT CALLBACK Toolbar::wndProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam) {
    if (message == WM_NCCREATE) {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        auto* self = static_cast<Toolbar*>(create->lpCreateParams);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(self));
        return TRUE;
    }

    auto* self = reinterpret_cast<Toolbar*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    if (!self) {
        return DefWindowProcW(hwnd, message, wparam, lparam);
    }

    switch (message) {
    case WM_COMMAND:
        SendMessageW(self->owner_, WM_COMMAND, wparam, lparam);
        return 0;
    case WM_PAINT:
        self->paint();
        return 0;
    default:
        return DefWindowProcW(hwnd, message, wparam, lparam);
    }
}

void Toolbar::paint() {
    PAINTSTRUCT ps{};
    HDC hdc = BeginPaint(hwnd_, &ps);
    RECT rc{};
    GetClientRect(hwnd_, &rc);
    HBRUSH bg = CreateSolidBrush(RGB(247, 249, 251));
    FillRect(hdc, &rc, bg);
    DeleteObject(bg);
    HPEN line = CreatePen(PS_SOLID, 1, RGB(210, 216, 222));
    HGDIOBJ oldPen = SelectObject(hdc, line);
    MoveToEx(hdc, 0, rc.bottom - 1, nullptr);
    LineTo(hdc, rc.right, rc.bottom - 1);
    SelectObject(hdc, oldPen);
    DeleteObject(line);
    EndPaint(hwnd_, &ps);
}

} // namespace nest
