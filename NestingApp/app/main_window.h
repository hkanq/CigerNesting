#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "bridge/corel_bridge.h"
#include "core/document.h"
#include "engine/nesting_engine.h"
#include "import/importer.h"
#include "ui/canvas_view.h"
#include "ui/left_panel.h"
#include "ui/progress_bar.h"
#include "ui/right_panel.h"
#include "ui/toolbar.h"
#include <string>

namespace nest {

class MainWindow {
public:
    bool create(HINSTANCE instance, int showCommand);
    HWND hwnd() const { return hwnd_; }

    static LRESULT CALLBACK wndProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam);

private:
    void onCreate(HINSTANCE instance);
    void onSize(int width, int height);
    void onCommand(int id);
    void openFile();
    void startNesting();
    void stopNesting();
    void updateSolverUi();
    void exportCorelSession();
    void applySettingsToDocument(const EngineSettings& settings);

    HWND hwnd_ = nullptr;
    HINSTANCE instance_ = nullptr;

    Toolbar toolbar_;
    ProgressBar progressBar_;
    LeftPanel leftPanel_;
    RightPanel rightPanel_;
    CanvasView canvas_;

    Document document_;
    Importer importer_;
    NestingEngine engine_;
    CorelBridge corelBridge_;
    SolverSnapshot latestSnapshot_;
};

} // namespace nest
