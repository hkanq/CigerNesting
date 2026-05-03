#include "core/math_utils.h"
#include "geometry/collision.h"
#include <iostream>
#include <string>
#include <vector>

namespace {

using namespace nest;

Ring boxRing(double x0, double y0, double x1, double y1, bool isHole = false) {
    Ring ring;
    ring.points = {
        {x0, y0},
        {x1, y0},
        {x1, y1},
        {x0, y1},
        {x0, y0}
    };
    ring.isHole = isHole;
    ring.winding = isHole ? -1 : 1;
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

Part trianglePart(Vec2 a, Vec2 b, Vec2 c) {
    Ring ring;
    ring.points = {a, b, c, a};
    return partFromRings({ring});
}

Part donutPart() {
    return partFromRings({
        boxRing(0.0, 0.0, 100.0, 100.0),
        boxRing(30.0, 30.0, 70.0, 70.0, true)
    });
}

Part bLikePart() {
    return partFromRings({
        boxRing(0.0, 0.0, 70.0, 130.0),
        boxRing(24.0, 22.0, 54.0, 52.0, true),
        boxRing(24.0, 78.0, 54.0, 108.0, true)
    });
}

Sheet customSheetWithZone(const Ring& zone, bool forbidden) {
    Sheet sheet;
    sheet.width = 120.0;
    sheet.height = 120.0;
    sheet.margin = 0.0;
    SheetProfile profile;
    profile.outerContour = boxRing(0.0, 0.0, 120.0, 120.0);
    profile.hasCustomProfile = true;
    if (forbidden) {
        profile.forbiddenZones.push_back(zone);
    } else {
        profile.holes.push_back(zone);
    }
    sheet.setProfile(profile);
    return sheet;
}

Sheet concaveLSheet() {
    Sheet sheet;
    sheet.width = 120.0;
    sheet.height = 120.0;
    sheet.margin = 0.0;
    SheetProfile profile;
    profile.outerContour.points = {
        {0.0, 0.0},
        {120.0, 0.0},
        {120.0, 40.0},
        {40.0, 40.0},
        {40.0, 120.0},
        {0.0, 120.0},
        {0.0, 0.0}
    };
    profile.hasCustomProfile = true;
    sheet.setProfile(profile);
    return sheet;
}

bool expect(bool condition, const char* name) {
    if (!condition) {
        std::cerr << "FAIL: " << name << "\n";
        return false;
    }
    std::cout << "PASS: " << name << "\n";
    return true;
}

} // namespace

int main() {
    constexpr double eps = 1e-6;
    bool ok = true;

    const Part square = boxPart(0.0, 0.0, 10.0, 10.0);
    Pose origin;
    Pose overlap;
    overlap.x = 5.0;
    overlap.y = 5.0;
    ok &= expect(partsCollide(square, origin, square, overlap, eps), "overlapping squares collide");

    Pose touching;
    touching.x = 10.0;
    ok &= expect(partsCollide(square, origin, square, touching, eps), "touching squares collide at tolerance");

    Pose separated;
    separated.x = 25.0;
    ok &= expect(!partsCollide(square, origin, square, separated, eps), "separate squares do not collide");

    const Part donut = donutPart();
    const Part small = boxPart(0.0, 0.0, 12.0, 12.0);
    Pose inHole;
    inHole.x = 44.0;
    inHole.y = 44.0;
    ok &= expect(!partsCollide(donut, origin, small, inHole, eps), "part inside donut hole is allowed");

    Pose inWall;
    inWall.x = 22.0;
    inWall.y = 44.0;
    ok &= expect(partsCollide(donut, origin, small, inWall, eps), "part hitting donut wall collides");

    const Part bLike = bLikePart();
    Pose inTopHole;
    inTopHole.x = 32.0;
    inTopHole.y = 86.0;
    ok &= expect(!partsCollide(bLike, origin, small, inTopHole, eps), "part inside B-like upper hole is allowed");

    Sheet rectangular;
    rectangular.width = 100.0;
    rectangular.height = 100.0;
    rectangular.margin = 0.0;
    Pose outsideSheet;
    outsideSheet.x = 95.0;
    outsideSheet.y = 95.0;
    ok &= expect(!isPartInsideSheet(square, outsideSheet, rectangular, eps), "part outside rectangular sheet is invalid");

    const Sheet holedSheet = customSheetWithZone(boxRing(40.0, 40.0, 80.0, 80.0, true), false);
    Pose inSheetHole;
    inSheetHole.x = 50.0;
    inSheetHole.y = 50.0;
    ok &= expect(!isPartInsideSheet(small, inSheetHole, holedSheet, eps), "part inside sheet hole is invalid");

    const Sheet forbiddenSheet = customSheetWithZone(boxRing(40.0, 40.0, 80.0, 80.0), true);
    ok &= expect(overlapsSheetHolesOrForbiddenZones(small, inSheetHole, forbiddenSheet, eps), "forbidden zone overlap is invalid");

    const Part longBox = boxPart(-10.0, -2.0, 10.0, 2.0);
    Pose rotatedMirrored;
    rotatedMirrored.x = 2.0;
    rotatedMirrored.y = 0.0;
    rotatedMirrored.angleRadians = degreesToRadians(35.0);
    rotatedMirrored.mirrored = true;
    ok &= expect(partsCollide(longBox, origin, longBox, rotatedMirrored, eps), "mirrored rotated pose collision is detected");

    const Sheet lSheet = concaveLSheet();
    const Part edgeLeavesConcaveSheet = trianglePart({30.0, 80.0}, {80.0, 30.0}, {30.0, 30.0});
    ok &= expect(!isPartInsideSheet(edgeLeavesConcaveSheet, origin, lSheet, eps), "concave sheet catches edge leaving and re-entering");

    const Part holeCrossing = boxPart(30.0, 50.0, 50.0, 70.0);
    ok &= expect(!isPartInsideSheet(holeCrossing, origin, holedSheet, eps), "sheet hole boundary crossing is invalid");

    ok &= expect(overlapsSheetHolesOrForbiddenZones(holeCrossing, origin, forbiddenSheet, eps), "forbidden zone boundary crossing is invalid");

    const ClearanceSettings clearance{5.0, 0.0, eps};
    Pose nearButSeparate;
    nearButSeparate.x = 13.0;
    ok &= expect(!partsRespectSpacing(square, origin, square, nearButSeparate, clearance), "clearance detects near separate parts");
    nearButSeparate.x = 16.0;
    ok &= expect(partsRespectSpacing(square, origin, square, nearButSeparate, clearance), "clearance accepts sufficiently separated parts");

    return ok ? 0 : 1;
}
