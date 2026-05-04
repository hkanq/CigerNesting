#include "engine/adaptive_optimizer.h"

#include "core/math_utils.h"
#include "engine/convergence.h"
#include "engine/free_space_analyzer.h"
#include "engine/frontier_analyzer.h"
#include "engine/layout_eval_cache.h"
#include "engine/layout_score.h"
#include "engine/pose_sampler.h"
#include "geometry/transformed_shape.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <unordered_map>

namespace nest {
namespace {

using LocalClock = std::chrono::steady_clock;

struct OperatorContext {
    const Document& document;
    const EngineSettings& settings;
    const LayoutEvalCache& cache;
    const std::vector<FreeSpaceCandidate>& freeSpace;
    const std::vector<FrontierCandidate>& frontiers;
    const std::vector<PartState>& partStates;
    size_t iteration = 0;
};

double elapsedSeconds(LocalClock::time_point started) {
    return std::chrono::duration<double>(LocalClock::now() - started).count();
}

double partFootprint(const Part& part) {
    const double boundsArea = part.localBounds.area();
    return part.area > 0.0 ? std::min(part.area, boundsArea > 0.0 ? boundsArea : part.area) : boundsArea;
}

bool samePose(const Pose& a, const Pose& b) {
    return std::abs(a.x - b.x) < 1e-9 &&
        std::abs(a.y - b.y) < 1e-9 &&
        std::abs(a.angleRadians - b.angleRadians) < 1e-9 &&
        a.mirrored == b.mirrored;
}

Pose poseAtCenter(const Part& part, double angle, bool mirrored, Vec2 center) {
    Pose orientation;
    orientation.angleRadians = angle;
    orientation.mirrored = mirrored;
    const AABB bounds = transformedBounds(part, orientation);
    Pose pose = orientation;
    pose.x = center.x - bounds.center().x;
    pose.y = center.y - bounds.center().y;
    return pose;
}

void appendCandidate(std::vector<CandidateMove>& out, PartId part, Pose pose, OperatorKind source, double estimate = 0.0) {
    for (const CandidateMove& existing : out) {
        if (existing.part == part && existing.source == source && !existing.isMultiPart() && samePose(existing.newPose, pose)) {
            return;
        }
    }
    CandidateMove move;
    move.part = part;
    move.newPose = pose;
    move.estimatedDeltaScore = estimate;
    move.source = source;
    out.push_back(std::move(move));
}

void appendMultiCandidate(
    std::vector<CandidateMove>& out,
    PartId primary,
    std::vector<PartId> parts,
    std::vector<Pose> poses,
    OperatorKind source) {
    if (parts.size() < 2 || parts.size() != poses.size()) {
        return;
    }
    CandidateMove move;
    move.part = primary;
    move.newPose = poses.front();
    move.source = source;
    move.parts = std::move(parts);
    move.newPoses = std::move(poses);
    out.push_back(std::move(move));
}

int xCompressionSign(PlacementStrategy strategy) {
    switch (strategy) {
    case PlacementStrategy::BottomRight:
    case PlacementStrategy::TopRight:
    case PlacementStrategy::RightToLeft:
        return 1;
    default:
        return -1;
    }
}

int yCompressionSign(PlacementStrategy strategy) {
    switch (strategy) {
    case PlacementStrategy::TopLeft:
    case PlacementStrategy::TopRight:
    case PlacementStrategy::TopToBottom:
        return 1;
    default:
        return -1;
    }
}

std::vector<double> limitedAngles(const EngineSettings& settings, const Pose& current, size_t limit) {
    std::vector<double> angles{current.angleRadians};
    if (!settings.allowRotation || settings.rotationMode == RotationMode::None) {
        return angles;
    }
    PoseSampler sampler;
    for (double angle : sampler.coarseRotationSamples(settings)) {
        bool exists = false;
        for (double existing : angles) {
            if (std::abs(existing - angle) < 1e-9) {
                exists = true;
                break;
            }
        }
        if (!exists) {
            angles.push_back(angle);
        }
        if (angles.size() >= limit) {
            break;
        }
    }
    return angles;
}

std::vector<bool> mirrorOptions(const EngineSettings& settings, const Pose& current, bool includeFlip) {
    std::vector<bool> mirrors{current.mirrored};
    if (settings.allowMirroring && includeFlip) {
        mirrors.push_back(!current.mirrored);
    }
    return mirrors;
}

size_t ringUsableCount(const std::vector<Vec2>& points) {
    if (points.size() > 2 && almostEqual(points.front(), points.back(), 1e-9)) {
        return points.size() - 1;
    }
    return points.size();
}

std::vector<Vec2> sampleRingPoints(const TransformedRing& ring, size_t stride, size_t limit) {
    std::vector<Vec2> points;
    const size_t count = ringUsableCount(ring.points);
    if (count == 0) {
        return points;
    }
    stride = std::max<size_t>(1, stride);
    for (size_t i = 0; i < count && points.size() < limit; i += stride) {
        points.push_back(ring.points[i]);
    }
    return points;
}

double sheetUsableArea(const Document& document, const EngineSettings& settings) {
    return std::max(1.0, (document.sheet.width - settings.margin * 2.0) * (document.sheet.height - settings.margin * 2.0));
}

class ContextOperator : public IOperator {
public:
    explicit ContextOperator(const OperatorContext& context) : context_(context) {}

protected:
    const OperatorContext& context_;
};

class CompressionOperator final : public ContextOperator {
public:
    using ContextOperator::ContextOperator;
    OperatorKind kind() const override { return OperatorKind::Compression; }

    void generateCandidates(const LayoutState& state, const PartState& part, std::vector<CandidateMove>& out) override {
        if (part.part >= state.poses.size() || part.part >= context_.document.parts.size()) {
            return;
        }
        const double minDim = std::max(1.0, std::min(context_.settings.sheetWidth, context_.settings.sheetHeight));
        const double base = std::max(0.5, std::min(32.0, minDim * 0.015));
        const std::array<double, 4> steps{base, base * 0.5, base * 2.0, std::max(0.25, context_.settings.partSpacing)};
        const Vec2 center{
            context_.document.sheet.origin.x + context_.document.sheet.width * 0.5,
            context_.document.sheet.origin.y + context_.document.sheet.height * 0.5
        };
        const AABB bounds = context_.cache.partBounds()[part.part];
        int sx = xCompressionSign(context_.settings.placementStrategy);
        int sy = yCompressionSign(context_.settings.placementStrategy);
        if (context_.settings.placementStrategy == PlacementStrategy::CenterOut ||
            context_.settings.placementStrategy == PlacementStrategy::OutsideIn) {
            sx = bounds.center().x < center.x ? 1 : -1;
            sy = bounds.center().y < center.y ? 1 : -1;
        }
        for (double step : steps) {
            Pose pose = state.poses[part.part];
            pose.x += static_cast<double>(sx) * step;
            appendCandidate(out, part.part, pose, kind());
            pose = state.poses[part.part];
            pose.y += static_cast<double>(sy) * step;
            appendCandidate(out, part.part, pose, kind());
        }
    }
};

class ContactPackingOperator final : public ContextOperator {
public:
    using ContextOperator::ContextOperator;
    OperatorKind kind() const override { return OperatorKind::ContactPacking; }

    void generateCandidates(const LayoutState& state, const PartState& part, std::vector<CandidateMove>& out) override {
        if (part.part >= state.poses.size() || part.part >= context_.document.parts.size()) {
            return;
        }
        const Pose base = state.poses[part.part];
        const size_t sampleStride = context_.document.parts.size() > 120 ? 4u : 2u;
        const std::vector<double> angles = limitedAngles(context_.settings, base, part.rotationSensitive ? 4u : 1u);
        const std::vector<bool> mirrors = mirrorOptions(context_.settings, base, part.mirrorSensitive);
        std::vector<size_t> owners = context_.cache.queryNeighbors(context_.cache.partBounds()[part.part].expanded(std::max(8.0, context_.settings.partSpacing + 8.0)), part.part);
        if (owners.empty()) {
            for (size_t i = 0; i < context_.document.parts.size() && owners.size() < 8; ++i) {
                if (i != part.part) {
                    owners.push_back(i);
                }
            }
        }
        const size_t ownerLimit = context_.settings.performanceProfile == PerformanceProfile::Maximum ? 8u : 5u;
        const size_t pointLimit = context_.settings.performanceProfile == PerformanceProfile::Maximum ? 8u : 5u;
        for (double angle : angles) {
            for (bool mirrored : mirrors) {
                Pose orientation;
                orientation.angleRadians = angle;
                orientation.mirrored = mirrored;
                const TransformedPart moving = transformPart(context_.document.parts[part.part], orientation, static_cast<int>(part.part));
                std::vector<Vec2> movingPoints;
                for (const TransformedRing& ring : moving.rings) {
                    std::vector<Vec2> sampled = sampleRingPoints(ring, sampleStride, pointLimit);
                    movingPoints.insert(movingPoints.end(), sampled.begin(), sampled.end());
                }
                for (size_t oi = 0; oi < owners.size() && oi < ownerLimit; ++oi) {
                    const size_t owner = owners[oi];
                    if (owner >= context_.cache.transformedParts().size()) {
                        continue;
                    }
                    for (const TransformedRing& ownerRing : context_.cache.transformedParts()[owner].rings) {
                        const std::vector<Vec2> ownerPoints = sampleRingPoints(ownerRing, sampleStride, pointLimit);
                        for (Vec2 ownerPoint : ownerPoints) {
                            for (Vec2 movingPoint : movingPoints) {
                                Pose pose;
                                pose.angleRadians = angle;
                                pose.mirrored = mirrored;
                                pose.x = ownerPoint.x - movingPoint.x;
                                pose.y = ownerPoint.y - movingPoint.y;
                                appendCandidate(out, part.part, pose, kind());
                                if (out.size() > 80) {
                                    return;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
};

class AnchorOperator : public ContextOperator {
public:
    AnchorOperator(const OperatorContext& context, OperatorKind kind) : ContextOperator(context), kind_(kind) {}
    OperatorKind kind() const override { return kind_; }

    void generateCandidates(const LayoutState& state, const PartState& part, std::vector<CandidateMove>& out) override {
        if (part.part >= state.poses.size() || part.part >= context_.document.parts.size()) {
            return;
        }
        const Pose base = state.poses[part.part];
        const std::vector<double> angles = limitedAngles(context_.settings, base, part.rotationSensitive ? 3u : 1u);
        size_t generated = 0;
        for (const FreeSpaceCandidate& anchor : context_.freeSpace) {
            if (!accepts(anchor.kind) || anchor.sourcePart == part.part) {
                continue;
            }
            for (double angle : angles) {
                Pose pose = poseAtCenter(context_.document.parts[part.part], angle, base.mirrored, anchor.anchor);
                appendCandidate(out, part.part, pose, kind_);
                ++generated;
                if (generated >= candidateLimit()) {
                    return;
                }
            }
        }
    }

private:
    bool accepts(FreeSpaceCandidateKind kind) const {
        switch (kind_) {
        case OperatorKind::GapFilling:
            return kind == FreeSpaceCandidateKind::UsedBoundsGap ||
                kind == FreeSpaceCandidateKind::SheetCorner ||
                kind == FreeSpaceCandidateKind::SheetBoundary;
        case OperatorKind::HoleFilling:
            return kind == FreeSpaceCandidateKind::PartHole;
        case OperatorKind::ConcavityFilling:
            return kind == FreeSpaceCandidateKind::Concavity;
        case OperatorKind::RegionRepack:
            return kind == FreeSpaceCandidateKind::UsedBoundsGap ||
                kind == FreeSpaceCandidateKind::ForbiddenZone;
        default:
            return false;
        }
    }

    size_t candidateLimit() const {
        return context_.settings.performanceProfile == PerformanceProfile::Maximum ? 18u :
            context_.settings.performanceProfile == PerformanceProfile::Balanced ? 12u : 7u;
    }

    OperatorKind kind_ = OperatorKind::GapFilling;
};

class SmallPartFillerOperator final : public AnchorOperator {
public:
    explicit SmallPartFillerOperator(const OperatorContext& context) : AnchorOperator(context, OperatorKind::SmallPartFiller) {}

    void generateCandidates(const LayoutState& state, const PartState& part, std::vector<CandidateMove>& out) override {
        const double average = context_.document.parts.empty() ? 1.0 : context_.document.totalPartArea() / static_cast<double>(context_.document.parts.size());
        if (part.part >= context_.document.parts.size() || partFootprint(context_.document.parts[part.part]) > average * 0.65) {
            return;
        }
        for (const FreeSpaceCandidate& anchor : context_.freeSpace) {
            if (anchor.sourcePart == part.part) {
                continue;
            }
            appendCandidate(out, part.part, poseAtCenter(context_.document.parts[part.part], state.poses[part.part].angleRadians, state.poses[part.part].mirrored, anchor.anchor), kind());
            if (out.size() > 48) {
                return;
            }
        }
    }
};

class FrontierOperator final : public ContextOperator {
public:
    using ContextOperator::ContextOperator;
    OperatorKind kind() const override { return OperatorKind::Frontier; }

    void generateCandidates(const LayoutState& state, const PartState& part, std::vector<CandidateMove>& out) override {
        if (part.part >= state.poses.size() || part.part >= context_.document.parts.size()) {
            return;
        }
        const size_t limit = context_.settings.performanceProfile == PerformanceProfile::Maximum ? 18u : 10u;
        size_t generated = 0;
        for (const FrontierCandidate& frontier : context_.frontiers) {
            if (frontier.sourcePart == part.part) {
                continue;
            }
            appendCandidate(out, part.part, poseAtCenter(context_.document.parts[part.part], state.poses[part.part].angleRadians, state.poses[part.part].mirrored, frontier.anchor), kind());
            if (++generated >= limit) {
                break;
            }
        }
    }
};

class RotationRefinementOperator final : public ContextOperator {
public:
    using ContextOperator::ContextOperator;
    OperatorKind kind() const override { return OperatorKind::RotationRefinement; }

    void generateCandidates(const LayoutState& state, const PartState& part, std::vector<CandidateMove>& out) override {
        if (!context_.settings.allowRotation || part.part >= state.poses.size()) {
            return;
        }
        const double coarseStep = degreesToRadians(std::max(0.001, context_.settings.rotationStepDegrees));
        const std::array<double, 8> steps{
            degreesToRadians(5.0),
            degreesToRadians(1.0),
            degreesToRadians(0.1),
            degreesToRadians(0.01),
            -degreesToRadians(5.0),
            -degreesToRadians(1.0),
            -degreesToRadians(0.1),
            -coarseStep
        };
        const size_t limit = context_.settings.performanceProfile == PerformanceProfile::Maximum ? steps.size() :
            context_.settings.performanceProfile == PerformanceProfile::Balanced ? 5u : 3u;
        for (size_t i = 0; i < limit; ++i) {
            Pose pose = state.poses[part.part];
            pose.angleRadians += steps[i];
            appendCandidate(out, part.part, pose, kind());
        }
    }
};

class MirrorOperator final : public ContextOperator {
public:
    using ContextOperator::ContextOperator;
    OperatorKind kind() const override { return OperatorKind::Mirror; }

    void generateCandidates(const LayoutState& state, const PartState& part, std::vector<CandidateMove>& out) override {
        if (!context_.settings.allowMirroring || part.part >= state.poses.size()) {
            return;
        }
        Pose pose = state.poses[part.part];
        pose.mirrored = !pose.mirrored;
        appendCandidate(out, part.part, pose, kind());
    }
};

class SwapOperator final : public ContextOperator {
public:
    using ContextOperator::ContextOperator;
    OperatorKind kind() const override { return OperatorKind::Swap; }

    void generateCandidates(const LayoutState& state, const PartState& part, std::vector<CandidateMove>& out) override {
        if (part.part >= state.poses.size()) {
            return;
        }
        std::vector<size_t> neighbors = context_.cache.queryNeighbors(context_.cache.partBounds()[part.part].expanded(64.0), part.part);
        if (neighbors.empty()) {
            for (size_t i = 0; i < state.poses.size() && neighbors.size() < 4; ++i) {
                if (i != part.part) {
                    neighbors.push_back(i);
                }
            }
        }
        const Vec2 centerA = context_.cache.partBounds()[part.part].center();
        const size_t limit = context_.settings.performanceProfile == PerformanceProfile::Maximum ? 5u : 3u;
        for (size_t i = 0; i < neighbors.size() && i < limit; ++i) {
            const size_t other = neighbors[i];
            if (other >= state.poses.size() || other >= context_.document.parts.size()) {
                continue;
            }
            const Vec2 centerB = context_.cache.partBounds()[other].center();
            Pose poseA = poseAtCenter(context_.document.parts[part.part], state.poses[part.part].angleRadians, state.poses[part.part].mirrored, centerB);
            Pose poseB = poseAtCenter(context_.document.parts[other], state.poses[other].angleRadians, state.poses[other].mirrored, centerA);
            appendMultiCandidate(out, part.part, {part.part, other}, {poseA, poseB}, kind());
        }
    }
};

class EjectionChainOperator final : public ContextOperator {
public:
    using ContextOperator::ContextOperator;
    OperatorKind kind() const override { return OperatorKind::EjectionChain; }

    void generateCandidates(const LayoutState& state, const PartState& part, std::vector<CandidateMove>& out) override {
        if (part.part >= state.poses.size() || context_.freeSpace.empty()) {
            return;
        }
        std::vector<size_t> neighbors = context_.cache.queryNeighbors(context_.cache.partBounds()[part.part].expanded(48.0), part.part);
        if (neighbors.empty()) {
            return;
        }
        const Vec2 sheetCenter{context_.settings.sheetWidth * 0.5, context_.settings.sheetHeight * 0.5};
        size_t generated = 0;
        for (const FreeSpaceCandidate& anchor : context_.freeSpace) {
            if (anchor.kind != FreeSpaceCandidateKind::PartHole && anchor.kind != FreeSpaceCandidateKind::Concavity && anchor.kind != FreeSpaceCandidateKind::UsedBoundsGap) {
                continue;
            }
            const size_t other = neighbors.front();
            if (other >= state.poses.size() || other >= context_.document.parts.size()) {
                continue;
            }
            Pose moved = poseAtCenter(context_.document.parts[part.part], state.poses[part.part].angleRadians, state.poses[part.part].mirrored, anchor.anchor);
            Pose pushed = state.poses[other];
            const Vec2 direction = context_.cache.partBounds()[other].center() - sheetCenter;
            const double len = std::max(1e-6, direction.length());
            const double step = std::max(8.0, context_.settings.partSpacing + 8.0);
            pushed.x += direction.x / len * step;
            pushed.y += direction.y / len * step;
            appendMultiCandidate(out, part.part, {part.part, other}, {moved, pushed}, kind());
            if (++generated >= 4) {
                break;
            }
        }
    }
};

class ClusterRepackOperator final : public ContextOperator {
public:
    using ContextOperator::ContextOperator;
    OperatorKind kind() const override { return OperatorKind::ClusterRepack; }

    void generateCandidates(const LayoutState& state, const PartState& part, std::vector<CandidateMove>& out) override {
        std::vector<size_t> cluster = context_.cache.queryNeighbors(context_.cache.partBounds()[part.part].expanded(72.0), part.part);
        if (cluster.size() < 2 || part.part >= state.poses.size()) {
            return;
        }
        cluster.insert(cluster.begin(), part.part);
        if (cluster.size() > 5) {
            cluster.resize(5);
        }
        AABB bounds;
        for (size_t index : cluster) {
            if (index < context_.cache.partBounds().size()) {
                bounds.include(context_.cache.partBounds()[index]);
            }
        }
        const Vec2 target = bounds.center();
        std::vector<Pose> poses;
        poses.reserve(cluster.size());
        const double step = context_.settings.performanceProfile == PerformanceProfile::Maximum ? 4.0 : 2.0;
        for (size_t index : cluster) {
            Pose pose = state.poses[index];
            const Vec2 center = context_.cache.partBounds()[index].center();
            const Vec2 delta = target - center;
            const double len = std::max(1e-6, delta.length());
            pose.x += delta.x / len * step;
            pose.y += delta.y / len * step;
            poses.push_back(pose);
        }
        appendMultiCandidate(out, part.part, std::move(cluster), std::move(poses), kind());
    }
};

class EscapeOperator final : public ContextOperator {
public:
    using ContextOperator::ContextOperator;
    OperatorKind kind() const override { return OperatorKind::Escape; }

    void generateCandidates(const LayoutState& state, const PartState& part, std::vector<CandidateMove>& out) override {
        if (part.part >= state.poses.size()) {
            return;
        }
        constexpr double golden = 2.39996322972865332;
        const double angle = static_cast<double>((part.part + 1) * (context_.iteration + 3)) * golden;
        const double step = context_.settings.performanceProfile == PerformanceProfile::Maximum ? 24.0 :
            context_.settings.performanceProfile == PerformanceProfile::Balanced ? 16.0 : 8.0;
        Pose pose = state.poses[part.part];
        pose.x += std::cos(angle) * step;
        pose.y += std::sin(angle) * step;
        appendCandidate(out, part.part, pose, kind());
    }
};

std::vector<std::unique_ptr<IOperator>> makeOperators(const OperatorContext& context) {
    std::vector<std::unique_ptr<IOperator>> operators;
    operators.push_back(std::make_unique<ContactPackingOperator>(context));
    operators.push_back(std::make_unique<CompressionOperator>(context));
    operators.push_back(std::make_unique<AnchorOperator>(context, OperatorKind::GapFilling));
    operators.push_back(std::make_unique<AnchorOperator>(context, OperatorKind::HoleFilling));
    operators.push_back(std::make_unique<AnchorOperator>(context, OperatorKind::ConcavityFilling));
    operators.push_back(std::make_unique<SmallPartFillerOperator>(context));
    operators.push_back(std::make_unique<SwapOperator>(context));
    operators.push_back(std::make_unique<EjectionChainOperator>(context));
    operators.push_back(std::make_unique<ClusterRepackOperator>(context));
    operators.push_back(std::make_unique<AnchorOperator>(context, OperatorKind::RegionRepack));
    operators.push_back(std::make_unique<RotationRefinementOperator>(context));
    operators.push_back(std::make_unique<MirrorOperator>(context));
    operators.push_back(std::make_unique<EscapeOperator>(context));
    operators.push_back(std::make_unique<FrontierOperator>(context));
    return operators;
}

bool operatorApplies(OperatorKind kind, const PartState& part, size_t noImprovementSteps) {
    switch (kind) {
    case OperatorKind::ContactPacking:
        return part.nearBoundary || part.highClearanceGap || part.inDenseRegion;
    case OperatorKind::Compression:
        return part.nearBoundary || part.highClearanceGap || part.inDenseRegion;
    case OperatorKind::GapFilling:
        return part.highClearanceGap || part.nearBoundary;
    case OperatorKind::HoleFilling:
        return part.candidateForHole;
    case OperatorKind::ConcavityFilling:
        return part.candidateForConcavity;
    case OperatorKind::SmallPartFiller:
        return part.candidateForHole || part.candidateForConcavity || part.highClearanceGap;
    case OperatorKind::Swap:
        return part.inDenseRegion || part.nearBoundary;
    case OperatorKind::EjectionChain:
        return part.candidateForHole || part.candidateForConcavity || noImprovementSteps > 4;
    case OperatorKind::ClusterRepack:
        return part.inDenseRegion || noImprovementSteps > 6;
    case OperatorKind::RegionRepack:
        return part.nearBoundary || part.highClearanceGap;
    case OperatorKind::RotationRefinement:
        return part.rotationSensitive;
    case OperatorKind::Mirror:
        return part.mirrorSensitive;
    case OperatorKind::Escape:
        return noImprovementSteps > 6;
    case OperatorKind::Frontier:
        return part.nearBoundary || part.highClearanceGap;
    }
    return false;
}

size_t targetLimit(const EngineSettings& settings, size_t partCount) {
    if (partCount <= 16) {
        return partCount;
    }
    size_t limit = settings.performanceProfile == PerformanceProfile::Maximum ? 48u :
        settings.performanceProfile == PerformanceProfile::Balanced ? 28u : 14u;
    if (partCount > 300) {
        limit = std::min<size_t>(limit, 24);
    }
    return std::min(partCount, limit);
}

size_t operatorBudget(const EngineSettings& settings, const OperatorStats& stats) {
    const double score = stats.schedulerScore();
    const size_t base = settings.performanceProfile == PerformanceProfile::Maximum ? 14u :
        settings.performanceProfile == PerformanceProfile::Balanced ? 9u : 5u;
    const size_t bonus = static_cast<size_t>(std::min(10.0, score * 0.10));
    return std::max<size_t>(3, base + bonus);
}

std::vector<PartState> analyzeParts(
    const Document& document,
    const EngineSettings& settings,
    const LayoutState& state,
    const LayoutEvalCache& cache,
    const std::vector<FreeSpaceCandidate>& freeSpace) {
    std::vector<PartState> parts;
    parts.reserve(state.poses.size());
    std::vector<char> colliding(state.poses.size(), 0);
    for (const CollisionPair& pair : state.collisionPairs) {
        if (pair.a < colliding.size()) {
            colliding[pair.a] = 1;
        }
        if (pair.b < colliding.size()) {
            colliding[pair.b] = 1;
        }
    }

    const AABB used = cache.usedBounds();
    const double boundaryEps = std::max(2.0, settings.partSpacing + settings.margin * 0.25);
    const double avgArea = document.parts.empty() ? 1.0 : document.totalPartArea() / static_cast<double>(document.parts.size());
    for (size_t i = 0; i < state.poses.size() && i < document.parts.size(); ++i) {
        PartState part;
        part.part = i;
        const AABB bounds = cache.partBounds()[i];
        const double footprint = partFootprint(document.parts[i]);
        part.hasCollision = colliding[i] != 0;
        part.nearBoundary = used.isValid() &&
            (std::abs(bounds.min.x - used.min.x) <= boundaryEps ||
             std::abs(bounds.max.x - used.max.x) <= boundaryEps ||
             std::abs(bounds.min.y - used.min.y) <= boundaryEps ||
             std::abs(bounds.max.y - used.max.y) <= boundaryEps);
        const std::vector<size_t> neighbors = cache.queryNeighbors(bounds.expanded(std::max(24.0, settings.partSpacing + 16.0)), i);
        part.neighborCount = neighbors.size();
        part.inDenseRegion = part.neighborCount >= (document.parts.size() > 120 ? 7u : 5u);
        part.highClearanceGap = part.neighborCount <= 2 || part.nearBoundary;
        const double aspect = bounds.isValid() ? std::max(bounds.width(), bounds.height()) / std::max(1.0, std::min(bounds.width(), bounds.height())) : 1.0;
        part.rotationSensitive = settings.allowRotation && aspect > 1.18;
        part.mirrorSensitive = settings.allowMirroring && (aspect > 1.05 || document.parts[i].rings.size() > 1);
        part.candidateForHole = footprint <= avgArea * 0.85;
        part.candidateForConcavity = footprint <= avgArea * 1.10;
        for (const FreeSpaceCandidate& candidate : freeSpace) {
            if (candidate.sourcePart == i) {
                continue;
            }
            if (candidate.kind == FreeSpaceCandidateKind::PartHole && candidate.featureBounds.area() >= footprint * 1.05) {
                part.candidateForHole = true;
            }
            if (candidate.kind == FreeSpaceCandidateKind::Concavity && candidate.featureBounds.area() >= footprint * 0.60) {
                part.candidateForConcavity = true;
            }
        }
        part.priority =
            (part.hasCollision ? 10000.0 : 0.0) +
            (part.nearBoundary ? 160.0 : 0.0) +
            (part.inDenseRegion ? 90.0 : 0.0) +
            (part.candidateForHole ? 75.0 : 0.0) +
            (part.candidateForConcavity ? 65.0 : 0.0) +
            (part.rotationSensitive ? 30.0 : 0.0) +
            (part.mirrorSensitive ? 25.0 : 0.0) +
            footprint / std::max(1.0, sheetUsableArea(document, settings)) * 40.0;
        parts.push_back(part);
    }
    return parts;
}

std::vector<PartState> selectTargets(const EngineSettings& settings, std::vector<PartState> parts) {
    std::stable_sort(parts.begin(), parts.end(), [](const PartState& a, const PartState& b) {
        if (std::abs(a.priority - b.priority) > 1e-9) {
            return a.priority > b.priority;
        }
        return a.part < b.part;
    });
    const size_t limit = targetLimit(settings, parts.size());
    if (parts.size() > limit) {
        parts.resize(limit);
    }
    return parts;
}

size_t layoutHash(const LayoutState& state) {
    size_t hash = 1469598103934665603ull;
    auto mix = [&](long long value) {
        hash ^= static_cast<size_t>(value) + 0x9e3779b97f4a7c15ull + (hash << 6) + (hash >> 2);
    };
    for (const Pose& pose : state.poses) {
        mix(static_cast<long long>(std::llround(pose.x * 10.0)));
        mix(static_cast<long long>(std::llround(pose.y * 10.0)));
        mix(static_cast<long long>(std::llround(pose.angleRadians * 10000.0)));
        mix(pose.mirrored ? 1 : 0);
    }
    return hash;
}

int noImprovementThreshold(const EngineSettings& settings, size_t partCount) {
    const int scale = static_cast<int>(std::max<size_t>(1, partCount / 100));
    switch (settings.performanceProfile) {
    case PerformanceProfile::Fast:
        return 8 + scale;
    case PerformanceProfile::Maximum:
        return 28 + scale * 3;
    case PerformanceProfile::Balanced:
    default:
        return 16 + scale * 2;
    }
}

int maxSteps(const EngineSettings& settings, size_t partCount) {
    const int scale = static_cast<int>(std::max<size_t>(1, partCount / 25));
    switch (settings.performanceProfile) {
    case PerformanceProfile::Fast:
        return 40 + scale;
    case PerformanceProfile::Maximum:
        return 170 + scale * 3;
    case PerformanceProfile::Balanced:
    default:
        return 90 + scale * 2;
    }
}

void classifyRejected(const DeltaEvaluation& evaluation, SolverStats& stats) {
    if (evaluation.collisionCount > 0) {
        ++stats.rejectedCollision;
    } else if (evaluation.spacingPenalty > 0.0) {
        ++stats.rejectedSpacing;
    } else if (evaluation.invalidPartCount > 0 || evaluation.sheetPenalty > 0.0) {
        ++stats.rejectedSheet;
    }
}

void classifyRejected(const MultiDeltaEvaluation& evaluation, SolverStats& stats) {
    if (evaluation.collisionCount > 0) {
        ++stats.rejectedCollision;
    } else if (evaluation.spacingPenalty > 0.0) {
        ++stats.rejectedSpacing;
    } else if (evaluation.invalidPartCount > 0 || evaluation.sheetPenalty > 0.0) {
        ++stats.rejectedSheet;
    }
}

void recordAccepted(OperatorKind kind, SolverStats& stats) {
    ++stats.acceptedMoves;
    switch (kind) {
    case OperatorKind::Compression:
        ++stats.compactionAccepted;
        break;
    case OperatorKind::GapFilling:
    case OperatorKind::HoleFilling:
    case OperatorKind::ConcavityFilling:
    case OperatorKind::ContactPacking:
    case OperatorKind::Frontier:
        ++stats.gapAccepted;
        break;
    case OperatorKind::SmallPartFiller:
        ++stats.smallFillerAccepted;
        break;
    case OperatorKind::Swap:
        ++stats.swapAccepted;
        break;
    case OperatorKind::EjectionChain:
        ++stats.chainAccepted;
        break;
    case OperatorKind::ClusterRepack:
        ++stats.clusterAccepted;
        break;
    case OperatorKind::RegionRepack:
        ++stats.regionRepackAccepted;
        break;
    case OperatorKind::RotationRefinement:
    case OperatorKind::Mirror:
        ++stats.ultraAccepted;
        break;
    case OperatorKind::Escape:
        ++stats.escapeAccepted;
        break;
    }
}

void recordAttempt(OperatorKind kind, SolverStats& stats) {
    switch (kind) {
    case OperatorKind::Swap:
        ++stats.swapAttempts;
        break;
    case OperatorKind::EjectionChain:
        ++stats.chainAttempts;
        break;
    case OperatorKind::ClusterRepack:
        ++stats.clusterAttempts;
        break;
    case OperatorKind::Escape:
        ++stats.escapeAttempts;
        break;
    case OperatorKind::Frontier:
        ++stats.frontierCandidates;
        break;
    default:
        break;
    }
}

bool highPotentialCavityOperator(OperatorKind kind) {
    return kind == OperatorKind::HoleFilling ||
        kind == OperatorKind::ConcavityFilling ||
        kind == OperatorKind::SmallPartFiller;
}

} // namespace

SolverStrategy strategyForOperator(OperatorKind kind) {
    switch (kind) {
    case OperatorKind::ContactPacking: return SolverStrategy::ContactPacking;
    case OperatorKind::Compression: return SolverStrategy::Compression;
    case OperatorKind::GapFilling: return SolverStrategy::GapFilling;
    case OperatorKind::HoleFilling: return SolverStrategy::HoleFilling;
    case OperatorKind::ConcavityFilling: return SolverStrategy::ConcavityFilling;
    case OperatorKind::SmallPartFiller: return SolverStrategy::SmallPartFiller;
    case OperatorKind::Swap: return SolverStrategy::Swap;
    case OperatorKind::EjectionChain: return SolverStrategy::EjectionChain;
    case OperatorKind::ClusterRepack: return SolverStrategy::ClusterRepack;
    case OperatorKind::RegionRepack: return SolverStrategy::RegionRepack;
    case OperatorKind::RotationRefinement: return SolverStrategy::RotationRefinement;
    case OperatorKind::Mirror: return SolverStrategy::Mirror;
    case OperatorKind::Escape: return SolverStrategy::Escape;
    case OperatorKind::Frontier: return SolverStrategy::Frontier;
    }
    return SolverStrategy::AdaptiveSearch;
}

LayoutState AdaptiveUnifiedOptimizer::optimize(
    const Document& document,
    const EngineSettings& settings,
    LayoutState initialValid,
    PenaltySystem& globalPenalties,
    const std::atomic_bool& stopRequested,
    SolverStats& stats,
    AdaptiveProgressCallback callback) const {
    operatorStats_.clear();
    const auto started = LocalClock::now();
    const double safetyLimit = effectiveSafetyTimeLimitSeconds(settings, document.parts.size());
    const double safetyGuard = std::min(4.0, std::max(0.15, static_cast<double>(document.parts.size()) * 0.006));
    const int stagnantThreshold = noImprovementThreshold(settings, document.parts.size());
    const int maxStepCount = maxSteps(settings, document.parts.size());

    LayoutScore scorer;
    PenaltySystem attemptPenalties;
    LayoutState current = scorer.evaluate(document, settings, initialValid.poses, &attemptPenalties, &globalPenalties, 0.10);
    if (!current.valid()) {
        return current;
    }

    LayoutState best = current;
    ConvergenceState convergence;
    convergence.lastBestScore = best.totalScore;
    convergence.lastAcceptedMoves = stats.acceptedMoves;
    std::unordered_map<size_t, int> seenLayouts;
    seenLayouts[layoutHash(current)] = 1;
    SolverStrategy currentStrategy = SolverStrategy::AdaptiveSearch;

    auto publish = [&]() {
        if (callback) {
            callback(currentStrategy, current, best, stats);
        }
    };
    auto timeExpired = [&]() {
        return elapsedSeconds(started) >= std::max(0.05, safetyLimit - safetyGuard);
    };
    publish();

    for (int step = 0; step < maxStepCount && !stopRequested.load() && !timeExpired(); ++step) {
        LayoutEvalCache cache;
        cache.rebuild(document, settings, current, &attemptPenalties, &globalPenalties, 0.10);

        FreeSpaceAnalyzer freeAnalyzer;
        FrontierAnalyzer frontierAnalyzer;
        const std::vector<FreeSpaceCandidate> freeSpace = freeAnalyzer.analyze(document, settings, current);
        const std::vector<FrontierCandidate> frontiers = frontierAnalyzer.analyze(document, settings, current);
        std::vector<PartState> partStates = analyzeParts(document, settings, current, cache, freeSpace);
        const std::vector<PartState> targets = selectTargets(settings, partStates);

        OperatorContext context{document, settings, cache, freeSpace, frontiers, partStates, static_cast<size_t>(step)};
        std::vector<std::unique_ptr<IOperator>> operators = makeOperators(context);
        std::stable_sort(operators.begin(), operators.end(), [&](const std::unique_ptr<IOperator>& a, const std::unique_ptr<IOperator>& b) {
            return operatorStats_[a->kind()].schedulerScore() > operatorStats_[b->kind()].schedulerScore();
        });

        CandidateMove bestMove;
        LayoutState bestMoveState;
        bool foundMove = false;
        bool acceptedWorse = false;
        double bestCandidateScore = std::numeric_limits<double>::max();

        for (const PartState& target : targets) {
            if (timeExpired() || stopRequested.load()) {
                break;
            }
            for (const std::unique_ptr<IOperator>& op : operators) {
                if (timeExpired() || stopRequested.load()) {
                    break;
                }
                if (!operatorApplies(op->kind(), target, static_cast<size_t>(convergence.noImprovementSteps))) {
                    continue;
                }
                std::vector<CandidateMove> candidates;
                op->generateCandidates(current, target, candidates);
                const size_t budget = operatorBudget(settings, operatorStats_[op->kind()]);
                if (candidates.size() > budget) {
                    candidates.resize(budget);
                }

                for (CandidateMove& candidate : candidates) {
                    if (stopRequested.load() || timeExpired()) {
                        break;
                    }
                    ++operatorStats_[candidate.source].attempts;
                    ++stats.evaluatedCandidates;
                    recordAttempt(candidate.source, stats);

                    bool deltaValid = false;
                    double deltaScore = std::numeric_limits<double>::max();
                    const bool forceFullCavityCheck = highPotentialCavityOperator(candidate.source);
                    if (candidate.isMultiPart()) {
                        MultiDeltaMove move;
                        move.partIndices = candidate.parts;
                        move.newPoses = candidate.newPoses;
                        for (size_t index : move.partIndices) {
                            if (index < current.poses.size()) {
                                move.oldPoses.push_back(current.poses[index]);
                            }
                        }
                        if (move.oldPoses.size() != move.partIndices.size()) {
                            continue;
                        }
                        MultiDeltaEvaluation delta = evaluateMultiMoveDelta(document, settings, current, cache, move);
                        deltaValid = delta.valid;
                        deltaScore = delta.totalScore;
                        if (!delta.valid) {
                            classifyRejected(delta, stats);
                            if (!forceFullCavityCheck) {
                                continue;
                            }
                        }
                    } else {
                        if (candidate.part >= current.poses.size() || samePose(current.poses[candidate.part], candidate.newPose)) {
                            continue;
                        }
                        DeltaMove move{candidate.part, current.poses[candidate.part], candidate.newPose};
                        DeltaEvaluation delta = evaluateMoveDelta(document, settings, current, cache, move);
                        deltaValid = delta.valid;
                        deltaScore = delta.totalScore;
                        if (!delta.valid) {
                            classifyRejected(delta, stats);
                            if (!forceFullCavityCheck) {
                                continue;
                            }
                        }
                    }

                    const double improvement = current.totalScore - deltaScore;
                    const bool allowWorse = candidate.source == OperatorKind::Escape &&
                        convergence.noImprovementSteps > stagnantThreshold / 2 &&
                        deltaScore < current.totalScore + std::max(100.0, std::abs(current.totalScore) * 0.02);
                    if (!deltaValid && !forceFullCavityCheck) {
                        continue;
                    }
                    if (deltaValid && improvement <= 1e-9 && !allowWorse && !forceFullCavityCheck) {
                        if (allowWorse) {
                            ++stats.rejectedWorseMoves;
                        }
                        continue;
                    }

                    std::vector<Pose> poses = current.poses;
                    if (candidate.isMultiPart()) {
                        for (size_t i = 0; i < candidate.parts.size(); ++i) {
                            if (candidate.parts[i] < poses.size()) {
                                poses[candidate.parts[i]] = candidate.newPoses[i];
                            }
                        }
                    } else {
                        poses[candidate.part] = candidate.newPose;
                    }
                    LayoutState verified = scorer.evaluate(document, settings, poses, &attemptPenalties, &globalPenalties, 0.10);
                    if (!verified.valid()) {
                        continue;
                    }
                    if (verified.totalScore + 1e-9 < bestCandidateScore || (!foundMove && allowWorse)) {
                        bestCandidateScore = verified.totalScore;
                        bestMoveState = std::move(verified);
                        bestMove = std::move(candidate);
                        foundMove = true;
                        acceptedWorse = bestMoveState.totalScore > current.totalScore + 1e-9;
                        currentStrategy = strategyForOperator(bestMove.source);
                    }
                }
            }
        }

        if (foundMove) {
            const double previousScore = current.totalScore;
            current = std::move(bestMoveState);
            const double improvement = std::max(0.0, previousScore - current.totalScore);
            ++operatorStats_[bestMove.source].accepted;
            operatorStats_[bestMove.source].totalImprovement += improvement / std::max(1.0, std::abs(previousScore));
            operatorStats_[bestMove.source].lastImprovementTime = elapsedSeconds(started);
            recordAccepted(bestMove.source, stats);
            if (acceptedWorse) {
                ++stats.acceptedWorseMoves;
            }
            if (current.valid() && current.totalScore + 1e-9 < best.totalScore) {
                best = current;
                ++stats.bestUpdates;
                convergence.noImprovementSteps = 0;
                convergence.lastBestScore = best.totalScore;
            } else {
                ++convergence.noImprovementSteps;
            }
        } else {
            ++convergence.noImprovementSteps;
        }

        const size_t hash = layoutHash(current);
        const int seen = ++seenLayouts[hash];
        convergence.repeatedLayouts = std::max(convergence.repeatedLayouts, seen);
        const size_t acceptedDelta = stats.acceptedMoves >= convergence.lastAcceptedMoves
            ? stats.acceptedMoves - convergence.lastAcceptedMoves
            : 0;
        convergence.lastAcceptedMoves = stats.acceptedMoves;
        if (acceptedDelta == 0) {
            ++convergence.lowAcceptanceSteps;
        } else {
            convergence.lowAcceptanceSteps = 0;
        }

        publish();

        const bool utilizationPlateau = best.utilization <= current.utilization + 1e-6 &&
            convergence.noImprovementSteps > stagnantThreshold;
        const bool repeated = convergence.repeatedLayouts >= 4;
        const bool lowAcceptance = convergence.lowAcceptanceSteps > stagnantThreshold / 2;
        if (convergence.noImprovementSteps > stagnantThreshold ||
            (utilizationPlateau && lowAcceptance) ||
            repeated) {
            break;
        }
    }

    LayoutState finalBest = scorer.evaluate(document, settings, best.poses, &attemptPenalties, &globalPenalties, 0.10);
    return finalBest.valid() ? finalBest : best;
}

} // namespace nest
