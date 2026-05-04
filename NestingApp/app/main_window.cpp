#include "app/main_window.h"

#include "localization/localization.h"
#include "resources/resource.h"
#include "ui/control_ids.h"
#include "ui/layout.h"
#include <commdlg.h>
#include <algorithm>
#include <sstream>
#include <utility>

namespace nest {
namespace {

const wchar_t* kMainWindowClassName = L"CigerNestingMainWindow";

void registerMainWindowClass(HINSTANCE instance) {
    static bool registered = false;
    if (registered) {
        return;
    }

    WNDCLASSEXW wc{};
    wc.cbSize = sizeof(wc);
    wc.hInstance = instance;
    wc.lpszClassName = kMainWindowClassName;
    wc.lpfnWndProc = MainWindow::wndProc;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hIcon = LoadIconW(instance, MAKEINTRESOURCEW(IDI_APP_ICON));
    wc.hIconSm = reinterpret_cast<HICON>(LoadImageW(instance, MAKEINTRESOURCEW(IDI_APP_ICON), IMAGE_ICON, 16, 16, LR_DEFAULTCOLOR));
    if (!wc.hIcon) {
        wc.hIcon = LoadIcon(nullptr, IDI_APPLICATION);
    }
    if (!wc.hIconSm) {
        wc.hIconSm = wc.hIcon;
    }
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    RegisterClassExW(&wc);
    registered = true;
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
    case SolverPhase::UltraRefinement: return TextId::UltraRefinement;
    case SolverPhase::FinalValidation: return TextId::FinalValidation;
    case SolverPhase::Done: return TextId::Done;
    case SolverPhase::Stopped: return TextId::Stopped;
    }
    return TextId::Idle;
}

std::wstring phaseLine(const SolverSnapshot& snapshot) {
    const auto& loc = Localization::instance();
    std::wostringstream out;
    out << loc.text(textIdForPhase(snapshot.phase)) << L"  " << static_cast<int>(snapshot.progress * 100.0) << L"%";
    return out.str();
}

std::wstring makeFileFilter() {
    const auto& loc = Localization::instance();
    std::wstring filter;
    auto append = [&](TextId label, const wchar_t* pattern) {
        filter += loc.text(label);
        filter.push_back(L'\0');
        filter += pattern;
        filter.push_back(L'\0');
    };
    append(TextId::FileFilterSupported, L"*.svg;*.dxf;*.plt;*.hpgl");
    append(TextId::FileFilterSvg, L"*.svg");
    append(TextId::FileFilterDxf, L"*.dxf");
    append(TextId::FileFilterPlt, L"*.plt;*.hpgl");
    append(TextId::FileFilterAll, L"*.*");
    filter.push_back(L'\0');
    return filter;
}

} // namespace

bool MainWindow::create(HINSTANCE instance, int showCommand) {
    instance_ = instance;
    registerMainWindowClass(instance);

    hwnd_ = CreateWindowExW(
        0,
        kMainWindowClassName,
        Localization::instance().text(TextId::AppTitle),
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        1280,
        820,
        nullptr,
        nullptr,
        instance,
        this);

    if (!hwnd_) {
        return false;
    }

    HICON largeIcon = LoadIconW(instance, MAKEINTRESOURCEW(IDI_APP_ICON));
    HICON smallIcon = reinterpret_cast<HICON>(LoadImageW(instance, MAKEINTRESOURCEW(IDI_APP_ICON), IMAGE_ICON, 16, 16, LR_DEFAULTCOLOR));
    if (largeIcon) {
        SendMessageW(hwnd_, WM_SETICON, ICON_BIG, reinterpret_cast<LPARAM>(largeIcon));
    }
    if (smallIcon) {
        SendMessageW(hwnd_, WM_SETICON, ICON_SMALL, reinterpret_cast<LPARAM>(smallIcon));
    }
    SetWindowTextW(hwnd_, Localization::instance().text(TextId::AppTitle));
    ShowWindow(hwnd_, showCommand);
    UpdateWindow(hwnd_);
    return true;
}

LRESULT CALLBACK MainWindow::wndProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam) {
    if (message == WM_NCCREATE) {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        auto* self = static_cast<MainWindow*>(create->lpCreateParams);
        self->hwnd_ = hwnd;
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(self));
        return TRUE;
    }

    auto* self = reinterpret_cast<MainWindow*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    if (!self) {
        return DefWindowProcW(hwnd, message, wparam, lparam);
    }

    switch (message) {
    case WM_CREATE:
        self->onCreate(reinterpret_cast<LPCREATESTRUCTW>(lparam)->hInstance);
        return 0;
    case WM_SIZE:
        self->onSize(LOWORD(lparam), HIWORD(lparam));
        return 0;
    case WM_COMMAND:
        self->onCommand(LOWORD(wparam));
        return 0;
    case WM_TIMER:
        if (wparam == uiid::timerSolver) {
            self->updateSolverUi();
            return 0;
        }
        break;
    case WM_DESTROY:
        self->engine_.stop();
        PostQuitMessage(0);
        return 0;
    default:
        break;
    }

    return DefWindowProcW(hwnd, message, wparam, lparam);
}

void MainWindow::onCreate(HINSTANCE instance) {
    toolbar_.create(hwnd_, instance);
    progressBar_.create(hwnd_, instance);
    leftPanel_.create(hwnd_, instance);
    rightPanel_.create(hwnd_, instance);
    canvas_.create(hwnd_, instance);
    canvas_.setDocument(&document_);
    leftPanel_.setFileInfo({}, 0);
    progressBar_.setValue(0.0);

    RECT rc{};
    GetClientRect(hwnd_, &rc);
    onSize(rc.right - rc.left, rc.bottom - rc.top);
}

void MainWindow::onSize(int width, int height) {
    const AppLayout layout = calculateLayout(width, height);
    toolbar_.move(layout.toolbar);
    leftPanel_.move(layout.leftPanel);
    rightPanel_.move(layout.rightPanel);
    canvas_.move(layout.canvas);

    RECT progressRect{width - 380, 17, width - 24, 38};
    if (progressRect.left < 760) {
        progressRect.left = 760;
    }
    if (progressRect.left < progressRect.right) {
        progressBar_.move(progressRect);
    }
}

void MainWindow::onCommand(int id) {
    const auto& loc = Localization::instance();
    switch (id) {
    case uiid::buttonOpen:
        openFile();
        break;
    case uiid::buttonSave:
        MessageBoxW(hwnd_, loc.text(TextId::SaveReady), loc.text(TextId::AppTitle), MB_OK | MB_ICONINFORMATION);
        break;
    case uiid::buttonStart:
        startNesting();
        break;
    case uiid::buttonStop:
        stopNesting();
        break;
    case uiid::buttonCorelConnect:
        MessageBoxW(hwnd_, loc.text(TextId::CorelBridgeReady), loc.text(TextId::AppTitle), MB_OK | MB_ICONINFORMATION);
        break;
    case uiid::buttonCorelExport:
        exportCorelSession();
        break;
    default:
        break;
    }
}

void MainWindow::openFile() {
    const auto& loc = Localization::instance();
    const std::wstring fileFilter = makeFileFilter();
    wchar_t fileName[MAX_PATH]{};
    OPENFILENAMEW ofn{};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = hwnd_;
    ofn.lpstrFile = fileName;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrFilter = fileFilter.c_str();
    ofn.lpstrTitle = loc.text(TextId::OpenFilePrompt);
    ofn.nFilterIndex = 1;
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

    if (!GetOpenFileNameW(&ofn)) {
        return;
    }

    const EngineSettings settings = rightPanel_.getSettings();
    ImportResult imported = importer_.importFile(fileName, settings.curveFlattenTolerance);
    if (!imported.ok) {
        MessageBoxW(hwnd_, loc.text(TextId::ImportFailed), loc.text(TextId::AppTitle), MB_OK | MB_ICONERROR);
        return;
    }

    document_.clear();
    document_.sourcePath = fileName;
    applySettingsToDocument(settings);
    for (auto& part : imported.parts) {
        document_.addPart(std::move(part));
    }

    latestSnapshot_ = {};
    canvas_.clearSnapshot();
    canvas_.setDocument(&document_);
    canvas_.fitToDocument();
    leftPanel_.setFileInfo(document_.sourcePath, document_.parts.size());
    leftPanel_.setStats(0, 0.0);
    progressBar_.setValue(0.0);
    toolbar_.setPhaseText(loc.text(TextId::ImportSuccess));
}

void MainWindow::applySettingsToDocument(const EngineSettings& settings) {
    document_.sheet.width = settings.sheetWidth;
    document_.sheet.height = settings.sheetHeight;
    document_.sheet.margin = settings.margin;
}

void MainWindow::startNesting() {
    const auto& loc = Localization::instance();
    if (document_.parts.empty()) {
        MessageBoxW(hwnd_, loc.text(TextId::OpenFileFirst), loc.text(TextId::Start), MB_OK | MB_ICONINFORMATION);
        return;
    }
    if (engine_.isRunning()) {
        return;
    }

    const EngineSettings settings = rightPanel_.getSettings();
    applySettingsToDocument(settings);
    canvas_.clearSnapshot();
    progressBar_.setValue(0.0);
    toolbar_.setPhaseText(loc.text(TextId::PrepareGeometry));

    engine_.setDocument(&document_);
    engine_.setSettings(settings);
    engine_.start();
    SetTimer(hwnd_, uiid::timerSolver, 33, nullptr);
}

void MainWindow::stopNesting() {
    engine_.requestStop();
    updateSolverUi();
}

void MainWindow::updateSolverUi() {
    latestSnapshot_ = engine_.getLatestSnapshot();
    if (latestSnapshot_.phase == SolverPhase::Idle && !latestSnapshot_.running && latestSnapshot_.currentPoses.empty() && latestSnapshot_.bestPoses.empty()) {
        return;
    }

    canvas_.setSnapshot(latestSnapshot_);
    progressBar_.setValue(latestSnapshot_.progress);
    toolbar_.setPhaseText(phaseLine(latestSnapshot_));
    leftPanel_.setStats(latestSnapshot_.collisionCount, latestSnapshot_.utilization);

    if (!latestSnapshot_.running && !engine_.isRunning()) {
        KillTimer(hwnd_, uiid::timerSolver);
    }
}

void MainWindow::exportCorelSession() {
    const auto& loc = Localization::instance();
    const SolverResult result = engine_.getBestResult();
    const std::wstring session = corelBridge_.exportResultSession(document_, result);
    (void)session;
    MessageBoxW(hwnd_, loc.text(TextId::CorelExportReady), loc.text(TextId::ExportToCorel), MB_OK | MB_ICONINFORMATION);
}

} // namespace nest
