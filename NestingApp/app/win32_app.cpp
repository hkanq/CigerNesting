#include "app/win32_app.h"

#include "app/main_window.h"

namespace nest {

int Win32App::run(HINSTANCE instance, int showCommand) {
    MainWindow window;
    if (!window.create(instance, showCommand)) {
        return 1;
    }

    MSG msg{};
    while (GetMessageW(&msg, nullptr, 0, 0) > 0) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
    return static_cast<int>(msg.wParam);
}

} // namespace nest
