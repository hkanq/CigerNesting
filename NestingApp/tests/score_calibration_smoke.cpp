#include "core/document.h"
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

Part rectPart(double w, double h) {
    Part part;
    Ring ring;
    ring.points = {{0.0, 0.0}, {w, 0.0}, {w, h}, {0.0, h}, {0.0, 0.0}};
    part.rings.push_back(std::move(ring));
    part.updateDerivedGeometry();
    return part;
}

EngineSettings makeSettings() {
    EngineSettings settings;
    settings.sheetWidth = 260.0;
    settings.sheetHeight = 180.0;
    settings.margin = 5.0;
    settings.partSpacing = 0.0;
    settings.collisionTolerance = 0.01;
    settings.performanceProfile = PerformanceProfile::Maximum;
    settings.qualityMode = QualityMode::MaxQuality;
    return settings;
}

Document makeDocument(const EngineSettings& settings) {
    Document document;
    document.sheet.width = settings.sheetWidth;
    document.sheet.height = settings.sheetHeight;
    document.sheet.margin = settings.margin;
    document.addPart(rectPart(40.0, 30.0));
    document.addPart(rectPart(36.0, 30.0));
    return document;
}

} // namespace

int main() {
    EngineSettings settings = makeSettings();
    Document document = makeDocument(settings);
    PenaltySystem penalties;
    LayoutScore scorer;

    std::vector<Pose> separated(2);
    separated[0].x = 20.0;
    separated[0].y = 20.0;
    separated[1].x = 140.0;
    separated[1].y = 20.0;
    LayoutState separatedState = scorer.evaluate(document, settings, separated, &penalties);

    std::vector<Pose> contact(2);
    contact[0].x = 20.0;
    contact[0].y = 20.0;
    contact[1].x = 60.0;
    contact[1].y = 20.0;
    LayoutState contactState = scorer.evaluate(document, settings, contact, &penalties);

    std::cout << "separated score=" << separatedState.totalScore
              << " util=" << separatedState.utilization
              << " contactReward=" << separatedState.contactReward
              << "\n";
    std::cout << "contact score=" << contactState.totalScore
              << " util=" << contactState.utilization
              << " contactReward=" << contactState.contactReward
              << "\n";

    bool ok = true;
    ok = expect("both layouts valid", separatedState.valid() && contactState.valid()) && ok;
    ok = expect("contact layout gets contact reward", contactState.contactReward > separatedState.contactReward) && ok;
    ok = expect("contact/compact layout scores better", contactState.totalScore < separatedState.totalScore) && ok;
    return ok ? 0 : 1;
}
