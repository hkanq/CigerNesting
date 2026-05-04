#include "core/document.h"
#include "engine/layout_eval_cache.h"
#include "engine/layout_score.h"
#include "engine/penalty_system.h"
#include "engine/pose_sampler.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace nest;
using Clock = std::chrono::steady_clock;

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
    ring.points = {{0.0, 0.0}, {w, 0.0}, {w, h * 0.42}, {w * 0.45, h * 0.42}, {w * 0.45, h}, {0.0, h}, {0.0, 0.0}};
    Part part;
    part.rings.push_back(std::move(ring));
    part.updateDerivedGeometry();
    return part;
}

Document makeDocument(size_t count) {
    Document doc;
    doc.sheet.width = std::max(240.0, static_cast<double>(count) * 14.0);
    doc.sheet.height = 260.0;
    doc.sheet.margin = 10.0;
    for (size_t i = 0; i < count; ++i) {
        if (i % 7 == 0) {
            doc.addPart(concavePart(14.0 + static_cast<double>(i % 5), 16.0));
        } else {
            doc.addPart(boxPart(12.0 + static_cast<double>(i % 4), 10.0 + static_cast<double>(i % 3)));
        }
    }
    return doc;
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

EngineSettings settings() {
    EngineSettings s;
    s.sheetWidth = 1000.0;
    s.sheetHeight = 260.0;
    s.margin = 10.0;
    s.partSpacing = 5.0;
    s.collisionTolerance = 0.001;
    s.performanceProfile = PerformanceProfile::Balanced;
    return s;
}

bool closeEnough(double a, double b, double tolerance = 1e-5) {
    return std::abs(a - b) <= tolerance * std::max(1.0, std::max(std::abs(a), std::abs(b)));
}

bool compareDeltaToFull(const char* name, const Document& doc, const EngineSettings& s, const LayoutState& current, const DeltaMove& move) {
    PenaltySystem penalties;
    LayoutEvalCache cache;
    cache.rebuild(doc, s, current, &penalties);
    const DeltaEvaluation delta = evaluateMoveDelta(doc, s, current, cache, move);
    std::vector<Pose> poses = current.poses;
    poses[move.partIndex] = move.newPose;
    const LayoutState full = LayoutScore{}.evaluate(doc, s, poses, &penalties);
    const bool ok = closeEnough(delta.totalScore, full.totalScore) &&
        delta.collisionCount == full.collisionCount &&
        delta.invalidPartCount == full.invalidPartCount &&
        closeEnough(delta.spacingPenalty, full.spacingPenalty) &&
        closeEnough(delta.sheetPenalty, full.sheetPenalty) &&
        closeEnough(delta.usedWidth, full.usedWidth) &&
        closeEnough(delta.usedHeight, full.usedHeight);
    std::cout << (ok ? "PASS: " : "FAIL: ") << name
              << " deltaScore=" << delta.totalScore
              << " fullScore=" << full.totalScore
              << " deltaCollisions=" << delta.collisionCount
              << " fullCollisions=" << full.collisionCount
              << " deltaInvalid=" << delta.invalidPartCount
              << " fullInvalid=" << full.invalidPartCount << "\n";
    return ok;
}

bool acceptedMoveUpdatesCache() {
    const EngineSettings s = settings();
    const Document doc = makeDocument(10);
    PenaltySystem penalties;
    LayoutState current = LayoutScore{}.evaluate(doc, s, rowPoses(doc, s), &penalties);
    LayoutEvalCache cache;
    cache.rebuild(doc, s, current, &penalties);

    DeltaMove first{2, current.poses[2], current.poses[2]};
    first.newPose.x += 3.0;
    const DeltaEvaluation firstDelta = evaluateMoveDelta(doc, s, current, cache, first);
    current.poses[2] = first.newPose;
    current = LayoutScore{}.evaluate(doc, s, current.poses, &penalties);
    cache.updateAfterAcceptedMove(doc, s, current, first, &penalties);

    DeltaMove second{4, current.poses[4], current.poses[4]};
    second.newPose.y += 2.0;
    const DeltaEvaluation secondDelta = evaluateMoveDelta(doc, s, current, cache, second);
    std::vector<Pose> poses = current.poses;
    poses[4] = second.newPose;
    const LayoutState full = LayoutScore{}.evaluate(doc, s, poses, &penalties);
    const bool ok = firstDelta.totalScore != 0.0 && closeEnough(secondDelta.totalScore, full.totalScore);
    std::cout << (ok ? "PASS: " : "FAIL: ") << "accepted move updates cache"
              << " secondDelta=" << secondDelta.totalScore
              << " full=" << full.totalScore << "\n";
    return ok;
}

bool deltaIsFasterThanFull() {
    EngineSettings s = settings();
    s.sheetWidth = 1700.0;
    const Document doc = makeDocument(100);
    PenaltySystem penalties;
    LayoutState current = LayoutScore{}.evaluate(doc, s, rowPoses(doc, s), &penalties);
    LayoutEvalCache cache;
    cache.rebuild(doc, s, current, &penalties);
    PoseSampler sampler;
    auto candidates = sampler.moveCandidates(doc, s, current.poses, 10, 99u, 0);
    if (candidates.size() > 96) {
        candidates.resize(96);
    }

    const auto deltaStart = Clock::now();
    double deltaSum = 0.0;
    for (const Pose& candidate : candidates) {
        deltaSum += evaluateMoveDelta(doc, s, current, cache, DeltaMove{10, current.poses[10], candidate}).totalScore;
    }
    const double deltaMs = std::chrono::duration<double, std::milli>(Clock::now() - deltaStart).count();

    const auto fullStart = Clock::now();
    double fullSum = 0.0;
    for (const Pose& candidate : candidates) {
        std::vector<Pose> poses = current.poses;
        poses[10] = candidate;
        fullSum += LayoutScore{}.evaluate(doc, s, poses, &penalties).totalScore;
    }
    const double fullMs = std::chrono::duration<double, std::milli>(Clock::now() - fullStart).count();

    const bool ok = deltaMs < fullMs * 0.75 && std::isfinite(deltaSum) && std::isfinite(fullSum);
    std::cout << (ok ? "PASS: " : "FAIL: ") << "100 part delta candidate evaluation is faster"
              << " deltaMs=" << deltaMs
              << " fullMs=" << fullMs
              << " candidates=" << candidates.size() << "\n";
    return ok;
}

} // namespace

int main() {
    const EngineSettings s = settings();
    const Document doc = makeDocument(10);
    PenaltySystem penalties;
    const LayoutState current = LayoutScore{}.evaluate(doc, s, rowPoses(doc, s), &penalties);

    DeltaMove move{3, current.poses[3], current.poses[3]};
    move.newPose.x += 4.0;
    bool ok = compareDeltaToFull("delta approximately equals full score", doc, s, current, move);

    DeltaMove collision{3, current.poses[3], current.poses[3]};
    collision.newPose = current.poses[2];
    ok = compareDeltaToFull("collision move detected by delta", doc, s, current, collision) && ok;

    DeltaMove spacing{3, current.poses[3], current.poses[3]};
    spacing.newPose.x = current.poses[2].x + doc.parts[2].localBounds.width() + 2.0;
    spacing.newPose.y = current.poses[2].y;
    ok = compareDeltaToFull("spacing violation detected by delta", doc, s, current, spacing) && ok;

    DeltaMove sheet{3, current.poses[3], current.poses[3]};
    sheet.newPose.x = 1.0;
    sheet.newPose.y = 1.0;
    ok = compareDeltaToFull("sheet invalid move detected by delta", doc, s, current, sheet) && ok;

    ok = acceptedMoveUpdatesCache() && ok;
    ok = deltaIsFasterThanFull() && ok;
    return ok ? 0 : 1;
}
