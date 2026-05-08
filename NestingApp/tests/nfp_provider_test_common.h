#pragma once

#include "core/document.h"
#include "core/math_utils.h"
#include "engine/engine_settings.h"
#include "engine/nfp_candidate_provider.h"
#include "engine/inner_fit_candidate_provider.h"
#include "geometry/clearance.h"
#include "geometry/collision.h"

#include <iostream>
#include <vector>

namespace nest::nfp_test {

inline bool expect(const char* name, bool condition) {
    std::cout << (condition ? "PASS: " : "QUALITY_FAIL: ") << name << "\n";
    return condition;
}

inline Part squarePart(const wchar_t* name, double w, double h, bool hole = false) {
    Part part;
    part.name = name;
    Ring ring;
    ring.isHole = hole;
    ring.points = {{0.0, 0.0}, {w, 0.0}, {w, h}, {0.0, h}};
    part.rings.push_back(ring);
    part.updateDerivedGeometry();
    return part;
}

inline Part donutPart() {
    Part part;
    part.name = L"donut";
    Ring outer;
    outer.points = {{0.0, 0.0}, {120.0, 0.0}, {120.0, 120.0}, {0.0, 120.0}};
    Ring hole;
    hole.isHole = true;
    hole.points = {{35.0, 35.0}, {85.0, 35.0}, {85.0, 85.0}, {35.0, 85.0}};
    part.rings = {outer, hole};
    part.updateDerivedGeometry();
    return part;
}

inline Part cShapePart() {
    Part part;
    part.name = L"c-shape";
    Ring ring;
    ring.points = {{0.0, 0.0}, {120.0, 0.0}, {120.0, 30.0}, {40.0, 30.0}, {40.0, 90.0}, {120.0, 90.0}, {120.0, 120.0}, {0.0, 120.0}};
    part.rings.push_back(ring);
    part.updateDerivedGeometry();
    return part;
}

inline Document simpleDocument() {
    Document document;
    document.sheet.width = 400.0;
    document.sheet.height = 300.0;
    document.sheet.margin = 0.0;
    document.addPart(squarePart(L"fixed", 80.0, 60.0));
    document.addPart(squarePart(L"moving", 40.0, 30.0));
    return document;
}

inline EngineSettings settings() {
    EngineSettings s;
    s.sheetWidth = 400.0;
    s.sheetHeight = 300.0;
    s.margin = 0.0;
    s.partSpacing = 0.0;
    s.collisionTolerance = 0.01;
    s.allowRotation = true;
    s.rotationMode = RotationMode::RightAngles;
    s.performanceProfile = PerformanceProfile::Maximum;
    s.qualityMode = QualityMode::MaxQuality;
    return s;
}

inline bool validCandidate(const Document& document, const EngineSettings& settings, const std::vector<Pose>& poses, size_t moving, const Pose& pose, const std::vector<size_t>& fixed) {
    if (!isPartInsideSheet(document.parts[moving], pose, document.sheet, settings.collisionTolerance) ||
        !partRespectsSheetClearance(document.parts[moving], pose, document.sheet, settings.margin, settings.collisionTolerance)) {
        return false;
    }
    for (size_t other : fixed) {
        if (partsCollide(document.parts[moving], pose, document.parts[other], poses[other], settings.collisionTolerance) ||
            !partsRespectClearance(document.parts[moving], pose, document.parts[other], poses[other], settings.partSpacing, settings.collisionTolerance)) {
            return false;
        }
    }
    return true;
}

inline ContactCandidateRequest requestFor(size_t moving, std::vector<size_t> fixed) {
    ContactCandidateRequest request;
    request.movingPart = moving;
    request.fixedParts = std::move(fixed);
    request.angles = {0.0, degreesToRadians(90.0)};
    request.mirrors = {false};
    request.ownerLimit = 8;
    request.perOwnerPointLimit = 8;
    request.candidateLimit = 64;
    request.regionAnchors = {{120.0, 80.0}, {40.0, 40.0}};
    return request;
}

} // namespace nest::nfp_test
