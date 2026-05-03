#include "import/svg_parser.h"

namespace nest::tests {

bool svgParserSmokeTest() {
    const char* svg = "<svg><polygon points=\"0,0 10,0 10,10 0,10\"/></svg>";
    return SvgParser{}.parseText(svg, 0.25).ok;
}

} // namespace nest::tests
