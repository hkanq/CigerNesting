#include "ui/layout.h"

#include <algorithm>

namespace nest {

int rectWidth(const RECT& rect) {
    return rect.right - rect.left;
}

int rectHeight(const RECT& rect) {
    return rect.bottom - rect.top;
}

AppLayout calculateLayout(int width, int height) {
    const int toolbarHeight = 54;
    const int leftWidth = std::min(260, std::max(210, width / 5));
    const int rightWidth = std::min(330, std::max(280, width / 4));

    AppLayout layout;
    layout.toolbar = {0, 0, width, toolbarHeight};
    layout.leftPanel = {0, toolbarHeight, leftWidth, height};
    layout.rightPanel = {width - rightWidth, toolbarHeight, width, height};
    layout.canvas = {leftWidth, toolbarHeight, width - rightWidth, height};
    return layout;
}

} // namespace nest
