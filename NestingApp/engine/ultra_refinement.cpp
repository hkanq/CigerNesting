#include "engine/ultra_refinement.h"

#include "core/math_utils.h"
#include "engine/layout_score.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

namespace nest {
namespace {

using Clock = std::chrono::steady_clock;

struct RotationStage {
    double rangeDegrees = 0.0;
    double stepDegrees = 1.0;
};

struct CandidatePose {
    Pose pose;
    bool mirrorChanged = false;
    double angleStepDegrees = 0.0;
};

struct CandidateResult {
    bool found = false;
    LayoutState state;
    size_t evaluated = 0;
    double angleStepDegrees = 0.0;
};

double elapsedSeconds(Clock::time_point started) {
    return std::chrono::duration<double>(Clock::now() - started).count();
}

double requestedMinimumStepDegrees(const EngineSettings& settings) {
    return std::max(0.001, settings.rotationStepDegrees);
}

double refinementBudgetSeconds(const EngineSettings& settings) {
    const double limit = std::max(0.25, settings.timeLimitSeconds);
    switch (settings.qualityMode) {
    case QualityMode::Fast:
        return std::min(0.35, std::max(0.05, limit * 0.06));
    case QualityMode::MaxQuality:
        return std::min(4.0, std::max(0.20, limit * 0.22));
    case QualityMode::Balanced:
    default:
        return std::min(1.5, std::max(0.10, limit * 0.15));
    }
}

std::vector<RotationStage> rotationStages(const EngineSettings& settings) {
    const double minStep = requestedMinimumStepDegrees(settings);
    std::vector<RotationStage> stages;
    if (!settings.allowRotation || settings.rotationMode == RotationMode::None) {
        return stages;
    }

    auto addStage = [&](double rangeDegrees, double nominalStepDegrees) {
        const double step = std::max(minStep, nominalStepDegrees);
        if (step <= rangeDegrees * 2.0 + 1e-9) {
            stages.push_back({rangeDegrees, step});
        }
    };

    if (settings.qualityMode != QualityMode::Fast) {
        addStage(10.0, 1.0);
    }
    addStage(5.0, 1.0);
    addStage(1.0, 0.1);
    if (settings.qualityMode == QualityMode::Balanced || settings.qualityMode == QualityMode::MaxQuality) {
        addStage(0.05, 0.01);
    }
    if (settings.qualityMode == QualityMode::MaxQuality) {
        addStage(0.005, minStep);
    }
    return stages;
}

size_t partLimitForQuality(QualityMode quality, size_t partCount) {
    switch (quality) {
    case QualityMode::Fast:
        return std::min<size_t>(partCount, 4);
    case QualityMode::MaxQuality:
        return std::min<size_t>(partCount, 32);
    case QualityMode::Balanced:
    default:
        return std::min<size_t>(partCount, 12);
    }
}

double signedTurn(Vec2 a, Vec2 b, Vec2 c) {
    return cross(b - a, c - b);
}

bool ringLooksConcave(const Ring& ring) {
    if (ring.points.size() < 5) {
        return false;
    }
    int firstSign = 0;
    for (size_t i = 0; i + 2 < ring.points.size(); ++i) {
        const double turn = signedTurn(ring.points[i], ring.points[i + 1], ring.points[i + 2]);
        if (std::abs(turn) < 1e-9) {
            continue;
        }
        const int sign = turn > 0.0 ? 1 : -1;
        if (firstSign == 0) {
            firstSign = sign;
        } else if (sign != firstSign) {
            return true;
        }
    }
    return false;
}

double complexityBias(const Part& part) {
    double bias = 0.0;
    for (const Ring& ring : part.rings) {
        if (ring.isHole) {
            bias += 25.0;
        } else if (ringLooksConcave(ring)) {
            bias += 15.0;
        }
    }
    return bias;
}

std::vector<size_t> prioritizedParts(const Document& document, const LayoutState& state, const PenaltySystem& penalties) {
    const size_t count = std::min(document.parts.size(), state.poses.size());
    std::vector<size_t> order(count);
    std::iota(order.begin(), order.end(), 0);

    AABB used;
    for (size_t i = 0; i < count; ++i) {
        used.include(transformedBounds(document.parts[i], state.poses[i]));
    }
    const double usedArea = std::max(1.0, used.area());
    const double edgeTolerance = 2.0;

    auto priority = [&](size_t index) {
        const Part& part = document.parts[index];
        const AABB bounds = transformedBounds(part, state.poses[index]);
        double value = 0.0;
        value += (part.area > 0.0 ? part.area : part.localBounds.area()) * 100.0 / std::max(1.0, document.totalPartArea());
        value += bounds.area() * 40.0 / usedArea;
        if (std::abs(bounds.min.x - used.min.x) <= edgeTolerance ||
            std::abs(bounds.max.x - used.max.x) <= edgeTolerance ||
            std::abs(bounds.min.y - used.min.y) <= edgeTolerance ||
            std::abs(bounds.max.y - used.max.y) <= edgeTolerance) {
            value += 50.0;
        }
        for (size_t other = 0; other < count; ++other) {
            if (other != index) {
                value += std::max(0.0, penalties.weight(index, other) - 1.0) * 20.0;
            }
        }
        value += complexityBias(part);
        return value;
    };

    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return priority(a) > priority(b);
    });
    return order;
}

int horizontalSign(PlacementStrategy strategy) {
    switch (strategy) {
    case PlacementStrategy::BottomRight:
    case PlacementStrategy::TopRight:
    case PlacementStrategy::RightToLeft:
        return 1;
    default:
        return -1;
    }
}

int verticalSign(PlacementStrategy strategy) {
    switch (strategy) {
    case PlacementStrategy::TopLeft:
    case PlacementStrategy::TopRight:
    case PlacementStrategy::TopToBottom:
        return 1;
    default:
        return -1;
    }
}

int signToward(double from, double to) {
    if (std::abs(to - from) < 1e-9) {
        return 0;
    }
    return to > from ? 1 : -1;
}

Vec2 compressionMicroDirection(const Document& document, const EngineSettings& settings, const Part& part, const Pose& pose) {
    if (settings.placementStrategy == PlacementStrategy::CenterOut ||
        settings.placementStrategy == PlacementStrategy::OutsideIn ||
        settings.placementStrategy == PlacementStrategy::UserPoints) {
        const AABB bounds = transformedBounds(part, pose);
        Vec2 target{document.sheet.origin.x + document.sheet.width * 0.5, document.sheet.origin.y + document.sheet.height * 0.5};
        if (settings.placementStrategy == PlacementStrategy::UserPoints && !document.sheet.getUserPlacementPoints().empty()) {
            target = document.sheet.getUserPlacementPoints().front();
        }
        return {
            static_cast<double>(signToward(bounds.center().x, target.x)),
            static_cast<double>(signToward(bounds.center().y, target.y))
        };
    }
    return {static_cast<double>(horizontalSign(settings.placementStrategy)), static_cast<double>(verticalSign(settings.placementStrategy))};
}

void addUniqueOffset(std::vector<Vec2>& offsets, Vec2 offset) {
    for (const Vec2& existing : offsets) {
        if (std::abs(existing.x - offset.x) < 1e-9 && std::abs(existing.y - offset.y) < 1e-9) {
            return;
        }
    }
    offsets.push_back(offset);
}

std::vector<Vec2> microOffsets(const Document& document, const EngineSettings& settings, const Part& part, const Pose& pose) {
    std::vector<double> units{0.0, 0.25, -0.25, 0.5, -0.5, 1.0, -1.0};
    const double spacingStep = std::max(0.25, std::min(1.0, settings.partSpacing * 0.25));
    units.push_back(spacingStep);
    units.push_back(-spacingStep);

    std::vector<Vec2> offsets;
    for (double dx : units) {
        for (double dy : units) {
            addUniqueOffset(offsets, {dx, dy});
        }
    }

    const Vec2 direction = compressionMicroDirection(document, settings, part, pose);
    addUniqueOffset(offsets, {direction.x * spacingStep, 0.0});
    addUniqueOffset(offsets, {0.0, direction.y * spacingStep});
    addUniqueOffset(offsets, {direction.x * spacingStep, direction.y * spacingStep});
    return offsets;
}

double normalizedAngle(double radians) {
    while (radians > pi) {
        radians -= twoPi;
    }
    while (radians < -pi) {
        radians += twoPi;
    }
    return radians;
}

void addUniqueAngle(std::vector<double>& angles, double angle) {
    angle = normalizedAngle(angle);
    for (double existing : angles) {
        if (std::abs(existing - angle) < 1e-12) {
            return;
        }
    }
    angles.push_back(angle);
}

std::vector<CandidatePose> buildCandidates(
    const Document& document,
    const EngineSettings& settings,
    const LayoutState& state,
    size_t partIndex,
    const RotationStage& stage) {
    std::vector<CandidatePose> candidates;
    if (partIndex >= document.parts.size() || partIndex >= state.poses.size()) {
        return candidates;
    }

    const Pose base = state.poses[partIndex];
    std::vector<double> angles;
    addUniqueAngle(angles, base.angleRadians);
    const int radius = static_cast<int>(std::ceil(stage.rangeDegrees / stage.stepDegrees));
    for (int i = -radius; i <= radius; ++i) {
        const double delta = static_cast<double>(i) * stage.stepDegrees;
        if (std::abs(delta) <= stage.rangeDegrees + 1e-9) {
            addUniqueAngle(angles, base.angleRadians + degreesToRadians(delta));
        }
    }

    std::vector<bool> mirrors{base.mirrored};
    if (settings.allowMirroring) {
        mirrors.push_back(!base.mirrored);
    }

    const auto offsets = microOffsets(document, settings, document.parts[partIndex], base);
    candidates.reserve(angles.size() * mirrors.size() * offsets.size());
    for (bool mirrored : mirrors) {
        for (double angle : angles) {
            for (const Vec2& offset : offsets) {
                Pose pose = base;
                pose.angleRadians = angle;
                pose.mirrored = mirrored;
                pose.x += offset.x;
                pose.y += offset.y;
                candidates.push_back({pose, mirrored != base.mirrored, stage.stepDegrees});
            }
        }
    }
    return candidates;
}

bool candidateImproves(const LayoutState& baseline, const LayoutState& trial, bool mirrorChanged) {
    if (!trial.valid()) {
        return false;
    }
    if (!baseline.valid()) {
        return true;
    }
    const double mirrorThreshold = mirrorChanged ? std::max(0.25, std::abs(baseline.totalScore) * 0.0005) : 1e-9;
    return trial.totalScore + mirrorThreshold < baseline.totalScore;
}

CandidateResult evaluateCandidates(
    const Document& document,
    const EngineSettings& settings,
    const PenaltySystem& penalties,
    const LayoutState& baseline,
    size_t partIndex,
    const std::vector<CandidatePose>& candidates,
    WorkerPool& workerPool,
    const std::atomic_bool& stopRequested) {
    CandidateResult overall;
    if (candidates.empty() || partIndex >= baseline.poses.size()) {
        return overall;
    }

    const size_t workerCount = std::max<size_t>(1, std::min(workerPool.threadCount(), candidates.size()));
    const size_t chunkSize = (candidates.size() + workerCount - 1) / workerCount;
    std::vector<std::future<CandidateResult>> futures;
    futures.reserve(workerCount);

    for (size_t worker = 0; worker < workerCount; ++worker) {
        const size_t begin = worker * chunkSize;
        const size_t end = std::min(candidates.size(), begin + chunkSize);
        if (begin >= end) {
            continue;
        }
        futures.push_back(workerPool.enqueue([&, begin, end]() {
            LayoutScore scorer;
            CandidateResult best;
            for (size_t i = begin; i < end && !stopRequested.load(); ++i) {
                std::vector<Pose> poses = baseline.poses;
                poses[partIndex] = candidates[i].pose;
                LayoutState trial = scorer.evaluate(document, settings, poses, &penalties);
                ++best.evaluated;
                if (candidateImproves(baseline, trial, candidates[i].mirrorChanged) &&
                    (!best.found || trial.totalScore < best.state.totalScore)) {
                    best.found = true;
                    best.angleStepDegrees = candidates[i].angleStepDegrees;
                    best.state = std::move(trial);
                }
            }
            return best;
        }));
    }

    for (auto& future : futures) {
        CandidateResult result = future.get();
        const size_t totalEvaluated = overall.evaluated + result.evaluated;
        overall.evaluated += result.evaluated;
        if (result.found && (!overall.found || result.state.totalScore < overall.state.totalScore)) {
            overall = std::move(result);
            overall.evaluated = totalEvaluated;
        }
    }
    return overall;
}

} // namespace

LayoutState UltraRefinement::refine(
    const Document& document,
    const EngineSettings& settings,
    LayoutState state,
    const PenaltySystem& penalties,
    WorkerPool& workerPool,
    const std::atomic_bool& stopRequested) const {
    UltraRefinementStats stats;
    LayoutScore scorer;
    state = scorer.evaluate(document, settings, state.poses, &penalties);
    stats.beforeScore = state.totalScore;
    stats.beforeUtilization = state.utilization;

    if (document.parts.empty() || state.poses.empty()) {
        stats.afterScore = state.totalScore;
        stats.afterUtilization = state.utilization;
        stats.collisionCount = state.collisionCount;
        stats.invalidPartCount = state.invalidPartCount;
        lastStats_ = stats;
        return state;
    }

    std::vector<RotationStage> stages = rotationStages(settings);
    if (stages.empty() && settings.allowMirroring) {
        stages.push_back({0.0, 1.0});
    }
    if (stages.empty() && !settings.allowMirroring) {
        stats.afterScore = state.totalScore;
        stats.afterUtilization = state.utilization;
        stats.collisionCount = state.collisionCount;
        stats.invalidPartCount = state.invalidPartCount;
        lastStats_ = stats;
        return state;
    }

    const auto started = Clock::now();
    const double budget = refinementBudgetSeconds(settings);
    const size_t partLimit = partLimitForQuality(settings.qualityMode, document.parts.size());
    std::vector<size_t> order = prioritizedParts(document, state, penalties);
    if (order.size() > partLimit) {
        order.resize(partLimit);
    }

    double minStepUsed = std::numeric_limits<double>::max();
    for (const RotationStage& stage : stages) {
        if (stopRequested.load() || elapsedSeconds(started) >= budget) {
            break;
        }
        bool stageAccepted = false;
        for (size_t partIndex : order) {
            if (stopRequested.load() || elapsedSeconds(started) >= budget) {
                break;
            }
            const auto candidates = buildCandidates(document, settings, state, partIndex, stage);
            if (!candidates.empty()) {
                minStepUsed = std::min(minStepUsed, stage.stepDegrees);
            }
            CandidateResult bestCandidate = evaluateCandidates(document, settings, penalties, state, partIndex, candidates, workerPool, stopRequested);
            stats.evaluatedCandidates += bestCandidate.evaluated;
            if (bestCandidate.found) {
                state = std::move(bestCandidate.state);
                ++stats.acceptedMoves;
                stageAccepted = true;
            }
        }
        if (!stageAccepted && settings.qualityMode == QualityMode::Fast) {
            break;
        }
    }

    state = scorer.evaluate(document, settings, state.poses, &penalties);
    stats.afterScore = state.totalScore;
    stats.afterUtilization = state.utilization;
    stats.angleStepMinUsedDegrees = minStepUsed == std::numeric_limits<double>::max() ? 0.0 : minStepUsed;
    stats.collisionCount = state.collisionCount;
    stats.invalidPartCount = state.invalidPartCount;
    lastStats_ = stats;
    return state;
}

} // namespace nest
