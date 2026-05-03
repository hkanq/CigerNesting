#include "core/document.h"
#include "core/math_utils.h"
#include "geometry/clearance.h"
#include "geometry/collision.h"
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace nest;

Ring boxRing(double x0, double y0, double x1, double y1) {
    Ring ring;
    ring.points = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}, {x0, y0}};
    return ring;
}

Part partFromRings(std::vector<Ring> rings) {
    Part part;
    part.rings = std::move(rings);
    part.updateDerivedGeometry();
    return part;
}

Part boxPart(double x0, double y0, double x1, double y1) {
    return partFromRings({boxRing(x0, y0, x1, y1)});
}

Part donutPart() {
    Ring hole = boxRing(30.0, 30.0, 70.0, 70.0);
    hole.isHole = true;
    return partFromRings({boxRing(0.0, 0.0, 100.0, 100.0), hole});
}

Sheet sheetWithHole() {
    Sheet sheet;
    sheet.width = 120.0;
    sheet.height = 120.0;
    sheet.margin = 0.0;
    SheetProfile profile;
    profile.outerContour = boxRing(0.0, 0.0, 120.0, 120.0);
    Ring hole = boxRing(50.0, 50.0, 70.0, 70.0);
    hole.isHole = true;
    profile.holes.push_back(hole);
    profile.hasCustomProfile = true;
    sheet.setProfile(profile);
    return sheet;
}

Sheet sheetWithForbiddenZone() {
    Sheet sheet;
    sheet.width = 140.0;
    sheet.height = 120.0;
    sheet.margin = 0.0;
    SheetProfile profile;
    profile.outerContour = boxRing(0.0, 0.0, 140.0, 120.0);
    profile.forbiddenZones.push_back(boxRing(60.0, 40.0, 82.0, 62.0));
    profile.hasCustomProfile = true;
    sheet.setProfile(profile);
    return sheet;
}

Pose pose(double x, double y, double angleDegrees = 0.0, bool mirrored = false) {
    Pose p;
    p.x = x;
    p.y = y;
    p.angleRadians = degreesToRadians(angleDegrees);
    p.mirrored = mirrored;
    return p;
}

bool expect(const char* name, bool condition) {
    std::cout << (condition ? "PASS: " : "FAIL: ") << name << "\n";
    return condition;
}

bool twoSquares(double gap, double required) {
    const Part a = boxPart(0.0, 0.0, 10.0, 10.0);
    const Part b = boxPart(0.0, 0.0, 10.0, 10.0);
    return partsRespectClearance(a, pose(0.0, 0.0), b, pose(10.0 + gap, 0.0), required, 1e-6);
}

bool donutHoleCloseToBoundaryIsInvalid() {
    const Part donut = donutPart();
    const Part small = boxPart(0.0, 0.0, 10.0, 10.0);
    return !partsCollide(donut, pose(0.0, 0.0), small, pose(58.0, 45.0), 1e-6) &&
        !partsRespectClearance(donut, pose(0.0, 0.0), small, pose(58.0, 45.0), 5.0, 1e-6);
}

bool donutHoleEnoughClearanceIsValid() {
    const Part donut = donutPart();
    const Part small = boxPart(0.0, 0.0, 10.0, 10.0);
    return !partsCollide(donut, pose(0.0, 0.0), small, pose(45.0, 45.0), 1e-6) &&
        partsRespectClearance(donut, pose(0.0, 0.0), small, pose(45.0, 45.0), 5.0, 1e-6);
}

bool sheetEdgeMargin(double x, double required) {
    Sheet sheet;
    sheet.width = 100.0;
    sheet.height = 100.0;
    sheet.margin = 0.0;
    const Part part = boxPart(0.0, 0.0, 20.0, 20.0);
    return partRespectsSheetClearance(part, pose(x, 40.0), sheet, required, 1e-6);
}

bool sheetHoleMarginInvalid() {
    const Sheet sheet = sheetWithHole();
    const Part part = boxPart(0.0, 0.0, 12.0, 12.0);
    return !partRespectsSheetClearance(part, pose(35.0, 54.0), sheet, 5.0, 1e-6);
}

bool forbiddenZoneMarginInvalid() {
    const Sheet sheet = sheetWithForbiddenZone();
    const Part part = boxPart(0.0, 0.0, 12.0, 12.0);
    return !partRespectsSheetClearance(part, pose(44.0, 44.0), sheet, 5.0, 1e-6);
}

bool rotatedMirroredClearanceWorks() {
    const Part a = boxPart(-10.0, -5.0, 10.0, 5.0);
    const Part b = boxPart(-8.0, -4.0, 8.0, 4.0);
    const Pose poseA = pose(50.0, 50.0, 30.0, false);
    const Pose nearPose = pose(71.0, 54.0, -15.0, true);
    const Pose farPose = pose(90.0, 60.0, -15.0, true);
    return !partsRespectClearance(a, poseA, b, nearPose, 5.0, 1e-6) &&
        partsRespectClearance(a, poseA, b, farPose, 5.0, 1e-6);
}

} // namespace

int main() {
    bool ok = true;
    ok = expect("two squares need 5mm but gap is 3mm", !twoSquares(3.0, 5.0)) && ok;
    ok = expect("two squares need 5mm and gap is 5mm", twoSquares(5.0, 5.0)) && ok;
    ok = expect("two squares need 5mm and gap is 8mm", twoSquares(8.0, 5.0)) && ok;
    ok = expect("donut hole placement near hole boundary violates clearance", donutHoleCloseToBoundaryIsInvalid()) && ok;
    ok = expect("donut hole placement with enough clearance is valid", donutHoleEnoughClearanceIsValid()) && ok;
    ok = expect("sheet edge margin 10mm but part is 5mm away", !sheetEdgeMargin(5.0, 10.0)) && ok;
    ok = expect("sheet edge margin 10mm and part is 10mm away", sheetEdgeMargin(10.0, 10.0)) && ok;
    ok = expect("sheet hole margin violation", sheetHoleMarginInvalid()) && ok;
    ok = expect("forbidden zone margin violation", forbiddenZoneMarginInvalid()) && ok;
    ok = expect("rotated and mirrored part clearance", rotatedMirroredClearanceWorks()) && ok;
    return ok ? 0 : 1;
}
