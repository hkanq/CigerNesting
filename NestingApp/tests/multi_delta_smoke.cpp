#include "core/document.h"
#include "engine/layout_eval_cache.h"
#include "engine/layout_score.h"
#include "engine/penalty_system.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

namespace {

using namespace nest;

Ring boxRing(double x0, double y0, double x1, double y1) {
    Ring ring;
    ring.points = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}, {x0, y0}};
    return ring;
}

Part boxPart(double w, double h) {
    Part part;
    part.rings.push_back(boxRing(0.0, 0.0, w, h));
    part.updateDerivedGeometry();
    return part;
}

Part concavePart(double w, double h) {
    Ring ring;
    ring.points = {{0.0, 0.0}, {w, 0.0}, {w, h * 0.45}, {w * 0.45, h * 0.45}, {w * 0.45, h}, {0.0, h}, {0.0, 0.0}};
    Part part;
    part.rings.push_back(std::move(ring));
    part.updateDerivedGeometry();
    return part;
}

Document makeDocument() {
    Document doc;
    doc.sheet.width = 420.0;
    doc.sheet.height = 260.0;
    doc.sheet.margin = 10.0;
    for (size_t i = 0; i < 14; ++i) {
        if (i % 5 == 0) {
            doc.addPart(concavePart(20.0 + static_cast<double>(i % 3), 18.0));
        } else {
            doc.addPart(boxPart(16.0 + static_cast<double>(i % 4), 12.0 + static_cast<double>(i % 3)));
        }
    }
    return doc;
}

EngineSettings makeSettings() {
    EngineSettings settings;
    settings.sheetWidth = 420.0;
    settings.sheetHeight = 260.0;
    settings.margin = 10.0;
    settings.partSpacing = 5.0;
    settings.collisionTolerance = 0.001;
    settings.performanceProfile = PerformanceProfile::Balanced;
    return settings;
}

std::vector<Pose> rowPoses(const Document& doc, const EngineSettings& settings) {
    std::vector<Pose> poses(doc.parts.size());
    double x = settings.margin;
    double y = settings.margin;
    double rowHeight = 0.0;
    for (size_t i = 0; i < doc.parts.size(); ++i) {
        const AABB bounds = doc.parts[i].localBounds;
        if (x + bounds.width() > doc.sheet.width - settings.margin) {
            x = settings.margin;
            y += rowHeight + settings.partSpacing;
            rowHeight = 0.0;
        }
        poses[i].x = x - bounds.min.x;
        poses[i].y = y - bounds.min.y;
        x += bounds.width() + settings.partSpacing;
        rowHeight = std::max(rowHeight, bounds.height());
    }
    return poses;
}

bool closeEnough(double a, double b, double tolerance = 1e-5) {
    return std::abs(a - b) <= tolerance * std::max(1.0, std::max(std::abs(a), std::abs(b)));
}

MultiDeltaMove buildMove(const LayoutState& current, const std::vector<size_t>& indices, const std::vector<Pose>& newPoses) {
    MultiDeltaMove move;
    for (size_t i = 0; i < indices.size(); ++i) {
        move.partIndices.push_back(indices[i]);
        move.oldPoses.push_back(current.poses[indices[i]]);
        move.newPoses.push_back(newPoses[i]);
    }
    return move;
}

bool compareMultiDeltaToFull(
    const char* name,
    const Document& doc,
    const EngineSettings& settings,
    const LayoutState& current,
    const MultiDeltaMove& move,
    bool requireCollision = false,
    bool requireSpacing = false,
    bool requireSheet = false) {
    PenaltySystem penalties;
    LayoutEvalCache cache;
    cache.rebuild(doc, settings, current, &penalties);
    MultiDeltaEvaluation delta = evaluateMultiMoveDelta(doc, settings, current, cache, move);

    std::vector<Pose> poses = current.poses;
    for (size_t i = 0; i < move.partIndices.size(); ++i) {
        poses[move.partIndices[i]] = move.newPoses[i];
    }
    const LayoutState full = LayoutScore{}.evaluate(doc, settings, poses, &penalties);

    const bool equivalent = closeEnough(delta.totalScore, full.totalScore) &&
        delta.collisionCount == full.collisionCount &&
        delta.invalidPartCount == full.invalidPartCount &&
        closeEnough(delta.spacingPenalty, full.spacingPenalty) &&
        closeEnough(delta.sheetPenalty, full.sheetPenalty);
    const bool expected = (!requireCollision || delta.collisionCount > 0) &&
        (!requireSpacing || delta.spacingPenalty > 0.0) &&
        (!requireSheet || delta.invalidPartCount > 0 || delta.sheetPenalty > 0.0);
    const bool ok = equivalent && expected;

    std::cout << (ok ? "PASS: " : "FAIL: ") << name
              << " deltaScore=" << delta.totalScore
              << " fullScore=" << full.totalScore
              << " deltaCollisions=" << delta.collisionCount
              << " fullCollisions=" << full.collisionCount
              << " deltaInvalid=" << delta.invalidPartCount
              << " fullInvalid=" << full.invalidPartCount
              << " deltaSpacing=" << delta.spacingPenalty
              << " fullSpacing=" << full.spacingPenalty
              << " affectedParts=" << delta.affectedPartCount
              << " affectedPairs=" << delta.affectedPairCount << "\n";
    return ok;
}

bool acceptedMultiMoveUpdatesCache(const Document& doc, const EngineSettings& settings, const LayoutState& initial) {
    PenaltySystem penalties;
    LayoutState current = initial;
    LayoutEvalCache cache;
    cache.rebuild(doc, settings, current, &penalties);

    std::vector<Pose> firstNew{current.poses[3], current.poses[4]};
    firstNew[0].x += 2.0;
    firstNew[1].y += 2.0;
    MultiDeltaMove first = buildMove(current, {3, 4}, firstNew);
    std::vector<Pose> poses = current.poses;
    poses[3] = firstNew[0];
    poses[4] = firstNew[1];
    current = LayoutScore{}.evaluate(doc, settings, poses, &penalties);
    cache.updateAfterAcceptedMultiMove(doc, settings, current, first, &penalties);

    std::vector<Pose> secondNew{current.poses[6], current.poses[7], current.poses[8]};
    secondNew[0].x += 3.0;
    secondNew[1].y += 1.5;
    secondNew[2].x += 1.0;
    MultiDeltaMove second = buildMove(current, {6, 7, 8}, secondNew);
    MultiDeltaEvaluation delta = evaluateMultiMoveDelta(doc, settings, current, cache, second);
    poses = current.poses;
    poses[6] = secondNew[0];
    poses[7] = secondNew[1];
    poses[8] = secondNew[2];
    const LayoutState full = LayoutScore{}.evaluate(doc, settings, poses, &penalties);
    const bool ok = closeEnough(delta.totalScore, full.totalScore) &&
        delta.collisionCount == full.collisionCount &&
        delta.invalidPartCount == full.invalidPartCount &&
        closeEnough(delta.spacingPenalty, full.spacingPenalty) &&
        closeEnough(delta.sheetPenalty, full.sheetPenalty);

    std::cout << (ok ? "PASS: " : "FAIL: ") << "accepted multi move updates cache"
              << " delta=" << delta.totalScore
              << " full=" << full.totalScore << "\n";
    return ok;
}

} // namespace

int main() {
    const Document doc = makeDocument();
    const EngineSettings settings = makeSettings();
    PenaltySystem penalties;
    const LayoutState current = LayoutScore{}.evaluate(doc, settings, rowPoses(doc, settings), &penalties);
    bool ok = current.valid();

    std::vector<Pose> swapNew{current.poses[2], current.poses[5]};
    std::swap(swapNew[0], swapNew[1]);
    ok = compareMultiDeltaToFull("swap move delta approximately equals full", doc, settings, current, buildMove(current, {2, 5}, swapNew)) && ok;

    std::vector<Pose> chainNew{current.poses[3], current.poses[4], current.poses[6]};
    chainNew[0].x += 2.0;
    chainNew[1].x += 2.0;
    chainNew[1].y += 1.0;
    chainNew[2].y += 2.0;
    ok = compareMultiDeltaToFull("chain move delta approximately equals full", doc, settings, current, buildMove(current, {3, 4, 6}, chainNew)) && ok;

    std::vector<Pose> clusterNew{current.poses[7], current.poses[8], current.poses[9], current.poses[10]};
    for (Pose& pose : clusterNew) {
        pose.x += 3.0;
        pose.y += 1.0;
    }
    ok = compareMultiDeltaToFull("cluster move delta approximately equals full", doc, settings, current, buildMove(current, {7, 8, 9, 10}, clusterNew)) && ok;

    std::vector<Pose> repackNew{current.poses[11], current.poses[12]};
    repackNew[0].x += 6.0;
    repackNew[1].y += 3.0;
    ok = compareMultiDeltaToFull("region repack style delta approximately equals full", doc, settings, current, buildMove(current, {11, 12}, repackNew)) && ok;

    std::vector<Pose> collisionNew{current.poses[1], current.poses[2]};
    collisionNew[0] = current.poses[0];
    collisionNew[1].x = current.poses[0].x + 1.0;
    collisionNew[1].y = current.poses[0].y;
    ok = compareMultiDeltaToFull("collision move detected by multi delta", doc, settings, current, buildMove(current, {1, 2}, collisionNew), true, false, false) && ok;

    std::vector<Pose> spacingNew{current.poses[3], current.poses[4]};
    spacingNew[0].x = current.poses[0].x + doc.parts[0].localBounds.width() + 2.0;
    spacingNew[0].y = current.poses[0].y;
    spacingNew[1].x += 2.0;
    ok = compareMultiDeltaToFull("spacing violation detected by multi delta", doc, settings, current, buildMove(current, {3, 4}, spacingNew), false, true, false) && ok;

    std::vector<Pose> sheetNew{current.poses[5], current.poses[6]};
    sheetNew[0].x = 1.0;
    sheetNew[0].y = 1.0;
    sheetNew[1].x = doc.sheet.width - 4.0;
    sheetNew[1].y = doc.sheet.height - 4.0;
    ok = compareMultiDeltaToFull("sheet violation detected by multi delta", doc, settings, current, buildMove(current, {5, 6}, sheetNew), false, false, true) && ok;

    ok = acceptedMultiMoveUpdatesCache(doc, settings, current) && ok;
    return ok ? 0 : 1;
}
