#include "ui/right_panel.h"

#include "localization/localization.h"
#include "ui/control_ids.h"
#include <cwchar>
#include <cstdlib>
#include <string>

namespace nest {
namespace {

const wchar_t* kRightPanelClassName = L"CigerNestingRightPanel";

void registerRightPanelClass(HINSTANCE instance) {
    static bool registered = false;
    if (registered) {
        return;
    }
    WNDCLASSW wc{};
    wc.hInstance = instance;
    wc.lpszClassName = kRightPanelClassName;
    wc.lpfnWndProc = RightPanel::wndProc;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = nullptr;
    RegisterClassW(&wc);
    registered = true;
}

HWND createLabel(HWND parent, HINSTANCE instance, const wchar_t* text) {
    return CreateWindowW(L"STATIC", text, WS_CHILD | WS_VISIBLE | SS_LEFT, 0, 0, 160, 20, parent, nullptr, instance, nullptr);
}

HMENU controlMenu(int id) {
    return reinterpret_cast<HMENU>(static_cast<INT_PTR>(id));
}

HWND createEdit(HWND parent, HINSTANCE instance, int id, const wchar_t* text) {
    return CreateWindowW(L"EDIT", text, WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL, 0, 0, 86, 24, parent, controlMenu(id), instance, nullptr);
}

} // namespace

bool RightPanel::create(HWND parent, HINSTANCE instance) {
    owner_ = parent;
    registerRightPanelClass(instance);
    hwnd_ = CreateWindowExW(0, kRightPanelClassName, L"", WS_CHILD | WS_VISIBLE | WS_CLIPCHILDREN, 0, 0, 300, 500, parent, nullptr, instance, this);
    if (!hwnd_) {
        return false;
    }
    createControls(instance);
    return true;
}

void RightPanel::createControls(HINSTANCE instance) {
    const auto& loc = Localization::instance();
    createLabel(hwnd_, instance, loc.text(TextId::SheetWidth));
    sheetWidthEdit_ = createEdit(hwnd_, instance, uiid::editSheetWidth, L"1000");
    createLabel(hwnd_, instance, loc.text(TextId::SheetHeight));
    sheetHeightEdit_ = createEdit(hwnd_, instance, uiid::editSheetHeight, L"600");
    createLabel(hwnd_, instance, loc.text(TextId::PartSpacing));
    spacingEdit_ = createEdit(hwnd_, instance, uiid::editSpacing, L"5");
    createLabel(hwnd_, instance, loc.text(TextId::Margin));
    marginEdit_ = createEdit(hwnd_, instance, uiid::editMargin, L"10");

    createLabel(hwnd_, instance, loc.text(TextId::PlacementStart));
    placementCombo_ = CreateWindowW(L"COMBOBOX", L"", WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST, 0, 0, 150, 220, hwnd_, controlMenu(uiid::comboPlacementStart), instance, nullptr);
    SendMessageW(placementCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(loc.text(TextId::BottomLeft)));
    SendMessageW(placementCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(loc.text(TextId::TopLeft)));
    SendMessageW(placementCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(loc.text(TextId::BottomRight)));
    SendMessageW(placementCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(loc.text(TextId::TopRight)));
    SendMessageW(placementCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(loc.text(TextId::LeftToRight)));
    SendMessageW(placementCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(loc.text(TextId::RightToLeft)));
    SendMessageW(placementCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(loc.text(TextId::TopToBottom)));
    SendMessageW(placementCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(loc.text(TextId::BottomToTop)));
    SendMessageW(placementCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(loc.text(TextId::CenterOut)));
    SendMessageW(placementCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(loc.text(TextId::OutsideIn)));
    SendMessageW(placementCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(loc.text(TextId::UserPoints)));
    SendMessageW(placementCombo_, CB_SETCURSEL, 0, 0);

    rotationCheck_ = CreateWindowW(L"BUTTON", loc.text(TextId::RotationEnabled), WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX, 0, 0, 140, 22, hwnd_, controlMenu(uiid::checkRotation), instance, nullptr);
    SendMessageW(rotationCheck_, BM_SETCHECK, BST_CHECKED, 0);

    createLabel(hwnd_, instance, loc.text(TextId::RotationMode));
    rotationModeCombo_ = CreateWindowW(L"COMBOBOX", L"", WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST, 0, 0, 150, 160, hwnd_, controlMenu(uiid::comboRotationMode), instance, nullptr);
    SendMessageW(rotationModeCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(loc.text(TextId::None)));
    SendMessageW(rotationModeCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(loc.text(TextId::RightAngles)));
    SendMessageW(rotationModeCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(loc.text(TextId::FortyFiveDegrees)));
    SendMessageW(rotationModeCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(loc.text(TextId::FixedStep)));
    SendMessageW(rotationModeCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(loc.text(TextId::ContinuousRefine)));
    SendMessageW(rotationModeCombo_, CB_SETCURSEL, 1, 0);

    createLabel(hwnd_, instance, loc.text(TextId::AnglePrecision));
    angleStepEdit_ = createEdit(hwnd_, instance, uiid::editAngleStep, L"1.0");

    mirrorCheck_ = CreateWindowW(L"BUTTON", loc.text(TextId::MirroringEnabled), WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX, 0, 0, 150, 22, hwnd_, controlMenu(uiid::checkMirroring), instance, nullptr);

    createLabel(hwnd_, instance, loc.text(TextId::QualityMode));
    qualityCombo_ = CreateWindowW(L"COMBOBOX", L"", WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST, 0, 0, 150, 140, hwnd_, controlMenu(uiid::comboQuality), instance, nullptr);
    SendMessageW(qualityCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(loc.text(TextId::Fast)));
    SendMessageW(qualityCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(loc.text(TextId::Balanced)));
    SendMessageW(qualityCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(loc.text(TextId::MaxQuality)));
    SendMessageW(qualityCombo_, CB_SETCURSEL, 1, 0);

    createLabel(hwnd_, instance, loc.text(TextId::TimeLimitSeconds));
    timeLimitEdit_ = createEdit(hwnd_, instance, uiid::editTimeLimit, L"30");
    createLabel(hwnd_, instance, loc.text(TextId::ThreadCount));
    threadEdit_ = createEdit(hwnd_, instance, uiid::editThreads, L"0");

    startButton_ = CreateWindowW(L"BUTTON", loc.text(TextId::Start), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 100, 30, hwnd_, controlMenu(uiid::buttonStart), instance, nullptr);
    stopButton_ = CreateWindowW(L"BUTTON", loc.text(TextId::Stop), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 100, 30, hwnd_, controlMenu(uiid::buttonStop), instance, nullptr);
    corelExportButton_ = CreateWindowW(L"BUTTON", loc.text(TextId::ExportToCorel), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 220, 30, hwnd_, controlMenu(uiid::buttonCorelExport), instance, nullptr);

    layoutControls();
}

void RightPanel::move(const RECT& rect) {
    if (hwnd_) {
        MoveWindow(hwnd_, rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top, TRUE);
        layoutControls();
    }
}

void RightPanel::layoutControls() {
    if (!hwnd_) {
        return;
    }
    const int labelX = 18;
    const int editX = 190;
    const int row = 34;
    const int editW = 80;

    const int labelYs[] = {52, 86, 120, 154, 188, 256, 290, 358, 392, 426};
    int labelIndex = 0;
    HWND child = GetWindow(hwnd_, GW_CHILD);
    while (child) {
        HWND next = GetWindow(child, GW_HWNDNEXT);
        wchar_t className[32]{};
        GetClassNameW(child, className, 32);
        const bool isStatic = wcscmp(className, L"Static") == 0;
        if (isStatic && labelIndex < static_cast<int>(sizeof(labelYs) / sizeof(labelYs[0]))) {
            MoveWindow(child, labelX, labelYs[labelIndex++], 166, 22, TRUE);
        }
        child = next;
    }

    int y = 48;
    const struct Row { HWND edit; } rows[] = {{sheetWidthEdit_}, {sheetHeightEdit_}, {spacingEdit_}, {marginEdit_}};
    for (const auto& r : rows) {
        MoveWindow(r.edit, editX, y, editW, 24, TRUE);
        y += row;
    }

    MoveWindow(placementCombo_, editX - 44, y, 124, 180, TRUE);
    y += row;
    MoveWindow(rotationCheck_, labelX, y + 4, 150, 22, TRUE);
    y += row;
    MoveWindow(rotationModeCombo_, editX - 28, y, 108, 120, TRUE);
    y += row;
    MoveWindow(angleStepEdit_, editX, y, editW, 24, TRUE);
    y += row;
    MoveWindow(mirrorCheck_, labelX, y + 4, 150, 22, TRUE);
    y += row;
    MoveWindow(qualityCombo_, editX - 28, y, 108, 100, TRUE);
    y += row;
    MoveWindow(timeLimitEdit_, editX, y, editW, 24, TRUE);
    y += row;
    MoveWindow(threadEdit_, editX, y, editW, 24, TRUE);
    y += row + 12;
    MoveWindow(startButton_, labelX, y, 110, 30, TRUE);
    MoveWindow(stopButton_, labelX + 126, y, 110, 30, TRUE);
    y += 42;
    MoveWindow(corelExportButton_, labelX, y, 236, 30, TRUE);
}

EngineSettings RightPanel::getSettings() const {
    EngineSettings settings;
    settings.sheetWidth = readDouble(sheetWidthEdit_, 1000.0);
    settings.sheetHeight = readDouble(sheetHeightEdit_, 600.0);
    settings.partSpacing = readDouble(spacingEdit_, 5.0);
    settings.margin = readDouble(marginEdit_, 10.0);
    settings.allowRotation = SendMessageW(rotationCheck_, BM_GETCHECK, 0, 0) == BST_CHECKED;
    settings.rotationStepDegrees = readDouble(angleStepEdit_, 1.0);
    settings.allowMirroring = SendMessageW(mirrorCheck_, BM_GETCHECK, 0, 0) == BST_CHECKED;
    settings.timeLimitSeconds = readDouble(timeLimitEdit_, 30.0);
    settings.cpuThreadCount = readInt(threadEdit_, 0);

    const LRESULT placement = SendMessageW(placementCombo_, CB_GETCURSEL, 0, 0);
    switch (placement) {
    case 1: settings.placementStrategy = PlacementStrategy::TopLeft; break;
    case 2: settings.placementStrategy = PlacementStrategy::BottomRight; break;
    case 3: settings.placementStrategy = PlacementStrategy::TopRight; break;
    case 4: settings.placementStrategy = PlacementStrategy::LeftToRight; break;
    case 5: settings.placementStrategy = PlacementStrategy::RightToLeft; break;
    case 6: settings.placementStrategy = PlacementStrategy::TopToBottom; break;
    case 7: settings.placementStrategy = PlacementStrategy::BottomToTop; break;
    case 8: settings.placementStrategy = PlacementStrategy::CenterOut; break;
    case 9: settings.placementStrategy = PlacementStrategy::OutsideIn; break;
    case 10: settings.placementStrategy = PlacementStrategy::UserPoints; break;
    default: settings.placementStrategy = PlacementStrategy::BottomLeft; break;
    }

    const LRESULT mode = SendMessageW(rotationModeCombo_, CB_GETCURSEL, 0, 0);
    switch (mode) {
    case 0: settings.rotationMode = RotationMode::None; break;
    case 2: settings.rotationMode = RotationMode::FortyFiveDegrees; break;
    case 3: settings.rotationMode = RotationMode::FixedStep; break;
    case 4: settings.rotationMode = RotationMode::ContinuousRefine; break;
    default: settings.rotationMode = RotationMode::RightAngles; break;
    }

    const LRESULT quality = SendMessageW(qualityCombo_, CB_GETCURSEL, 0, 0);
    if (quality == 0) {
        settings.qualityMode = QualityMode::Fast;
        settings.performanceProfile = PerformanceProfile::Fast;
    } else if (quality == 2) {
        settings.qualityMode = QualityMode::MaxQuality;
        settings.performanceProfile = PerformanceProfile::Maximum;
    } else {
        settings.qualityMode = QualityMode::Balanced;
        settings.performanceProfile = PerformanceProfile::Balanced;
    }
    return settings;
}

double RightPanel::readDouble(HWND edit, double fallback) const {
    wchar_t buffer[64]{};
    GetWindowTextW(edit, buffer, 64);
    wchar_t* end = nullptr;
    const double value = std::wcstod(buffer, &end);
    return end == buffer ? fallback : value;
}

int RightPanel::readInt(HWND edit, int fallback) const {
    wchar_t buffer[64]{};
    GetWindowTextW(edit, buffer, 64);
    wchar_t* end = nullptr;
    const long value = std::wcstol(buffer, &end, 10);
    return end == buffer ? fallback : static_cast<int>(value);
}

LRESULT CALLBACK RightPanel::wndProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam) {
    if (message == WM_NCCREATE) {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        auto* self = static_cast<RightPanel*>(create->lpCreateParams);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(self));
        return TRUE;
    }

    auto* self = reinterpret_cast<RightPanel*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    if (!self) {
        return DefWindowProcW(hwnd, message, wparam, lparam);
    }

    switch (message) {
    case WM_COMMAND:
        if (LOWORD(wparam) == uiid::buttonStart || LOWORD(wparam) == uiid::buttonStop || LOWORD(wparam) == uiid::buttonCorelExport) {
            SendMessageW(self->owner_, WM_COMMAND, wparam, lparam);
            return 0;
        }
        break;
    case WM_PAINT:
        self->paint();
        return 0;
    default:
        break;
    }
    return DefWindowProcW(hwnd, message, wparam, lparam);
}

void RightPanel::paint() {
    PAINTSTRUCT ps{};
    HDC hdc = BeginPaint(hwnd_, &ps);
    RECT rc{};
    GetClientRect(hwnd_, &rc);
    HBRUSH bg = CreateSolidBrush(RGB(246, 247, 243));
    FillRect(hdc, &rc, bg);
    DeleteObject(bg);

    SetBkMode(hdc, TRANSPARENT);
    SetTextColor(hdc, RGB(35, 45, 55));
    RECT title{18, 18, rc.right - 18, 40};
    DrawTextW(hdc, Localization::instance().text(TextId::NestingSettings), -1, &title, DT_LEFT | DT_TOP | DT_SINGLELINE);

    HPEN line = CreatePen(PS_SOLID, 1, RGB(216, 220, 210));
    HGDIOBJ oldPen = SelectObject(hdc, line);
    MoveToEx(hdc, 0, 0, nullptr);
    LineTo(hdc, 0, rc.bottom);
    SelectObject(hdc, oldPen);
    DeleteObject(line);
    EndPaint(hwnd_, &ps);
}

} // namespace nest
