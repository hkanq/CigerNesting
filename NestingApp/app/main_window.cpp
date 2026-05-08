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

std::wstring phaseLine(const SolverSnapshot& snapshot) {
    const auto& loc = Localization::instance();
    std::wostringstream out;
    const bool terminal = snapshot.phase == SolverPhase::Done ||
        snapshot.phase == SolverPhase::NoValidLayout ||
        snapshot.phase == SolverPhase::Failed ||
        snapshot.phase == SolverPhase::Stopped;
    if (terminal) {
        out << loc.text(textIdForPhase(snapshot.phase));
    } else {
        out << loc.text(TextId::ActiveMoves) << L": " << activeMovesText(snapshot);
    }
    out << L"  " << static_cast<int>(snapshot.progress * 100.0) << L"%";
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

void attachMainMenu(HWND hwnd) {
    const auto& loc = Localization::instance();
    HMENU mainMenu = CreateMenu();
    HMENU fileMenu = CreatePopupMenu();
    HMENU integrationMenu = CreatePopupMenu();
    HMENU settingsMenu = CreatePopupMenu();

    AppendMenuW(fileMenu, MF_STRING, uiid::buttonOpen, loc.text(TextId::FileOpen));
    AppendMenuW(fileMenu, MF_STRING, uiid::buttonSave, loc.text(TextId::Save));
    AppendMenuW(integrationMenu, MF_STRING, uiid::buttonCorelConnect, loc.text(TextId::CorelConnection));
    AppendMenuW(integrationMenu, MF_STRING, uiid::buttonCorelExport, loc.text(TextId::ExportToCorel));

    std::wstring autoLimit = std::wstring(loc.text(TextId::TimeLimitSeconds)) + L": 0 / Auto Convergence";
    AppendMenuW(settingsMenu, MF_STRING, uiid::menuSettingsSafetyCap, autoLimit.c_str());
    AppendMenuW(settingsMenu, MF_SEPARATOR, 0, nullptr);
    AppendMenuW(settingsMenu, MF_STRING, uiid::menuSettingsCpu, loc.text(TextId::ThreadMaximum));
    AppendMenuW(settingsMenu, MF_STRING, uiid::menuSettingsQuality, loc.text(TextId::MaxQuality));

    AppendMenuW(mainMenu, MF_POPUP, reinterpret_cast<UINT_PTR>(fileMenu), loc.text(TextId::FileMenu));
    AppendMenuW(mainMenu, MF_POPUP, reinterpret_cast<UINT_PTR>(integrationMenu), loc.text(TextId::Integrations));
    AppendMenuW(mainMenu, MF_POPUP, reinterpret_cast<UINT_PTR>(settingsMenu), loc.text(TextId::Settings));
    SetMenu(hwnd, mainMenu);
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
    attachMainMenu(hwnd_);
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
    case uiid::menuSettingsSafetyCap:
        MessageBoxW(hwnd_, L"Zaman aşımı şu anda 0: motor Auto Convergence ile durur. Pozitif değerler sonraki ayar diyalogunda yalnızca güvenlik üst sınırı olarak düzenlenecek.", loc.text(TextId::Settings), MB_OK | MB_ICONINFORMATION);
        break;
    case uiid::menuSettingsCpu:
        MessageBoxW(hwnd_, L"CPU kullanımı varsayılan olarak Maximum / tüm çekirdeklerdir. Çekirdek seçimi sağ paneldeki açılır listeden yapılır.", loc.text(TextId::Settings), MB_OK | MB_ICONINFORMATION);
        break;
    case uiid::menuSettingsQuality:
        MessageBoxW(hwnd_, L"Varsayılan kalite Maximum'dur. Fast yalnızca hızlı önizleme, Balanced orta kalite, Maximum endüstriyel çözüm modudur.", loc.text(TextId::Settings), MB_OK | MB_ICONINFORMATION);
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
