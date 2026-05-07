#include "core/document.h"
#include "engine/empty_space_map.h"
#include "engine/engine_settings.h"
#include "engine/layout_score.h"
#include "engine/penalty_system.h"

#include <iostream>
#include <utility>

namespace {
using namespace nest;

bool expect(const char* name, bool condition) {
    std::cout << (condition ? "PASS: " : "FAIL: ") << name << "\n";
    return condition;
}

Part rect(double w, double h) {
    Part part;
    Ring ring;
    ring.points = {{0.0, 0.0}, {w, 0.0}, {w, h}, {0.0, h}, {0.0, 0.0}};
    part.rings.push_back(std::move(ring));
    part.updateDerivedGeometry();
    return part;
}

} // namespace

int main() {
    EngineSettings settings;
    settings.sheetWidth = 260.0;
    settings.sheetHeight = 160.0;
    settings.margin = 5.0;
    settings.partSpacing = 0.0;
    settings.performanceProfile = PerformanceProfile::Maximum;

    Document document;
    document.sheet.width = settings.sheetWidth;
    document.sheet.height = settings.sheetHeight;
    document.sheet.margin = settings.margin;
    document.addPart(rect(40.0, 40.0));
    document.addPart(rect(40.0, 40.0));
    document.addPart(rect(35.0, 35.0));
    std::vector<Pose> poses(3);
    poses[0].x = 10.0; poses[0].y = 10.0;
    poses[1].x = 55.0; poses[1].y = 10.0;
    poses[2].x = 10.0; poses[2].y = 55.0;
    PenaltySystem penalties;
    LayoutState state = LayoutScore{}.evaluate(document, settings, poses, &penalties);
    EmptySpaceMap map = EmptySpaceAnalyzer{}.analyze(document, settings, state, 32, 24);
    std::cout << "emptyRegion total=" << map.totalEmptyArea
              << " largest=" << map.largestRegionArea
              << " regions=" << map.regions.size()
              << " fillable=" << map.fillableRegionCount(300.0)
              << "\n";
    bool ok = true;
    ok = expect("layout valid", state.valid()) && ok;
    ok = expect("empty regions detected", !map.regions.empty()) && ok;
    ok = expect("largest region is meaningful", map.largestRegionArea > 300.0) && ok;
    ok = expect("fillable gaps counted", map.fillableRegionCount(300.0) > 0) && ok;
    return ok ? 0 : 1;
}
