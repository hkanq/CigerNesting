#include "engine/adaptive_optimizer.h"

#include "core/math_utils.h"
#include "engine/convergence.h"
#include "engine/empty_space_map.h"
#include "engine/free_space_analyzer.h"
#include "engine/frontier_analyzer.h"
#include "engine/layout_eval_cache.h"
#include "engine/layout_score.h"
#include "engine/layout_score_components.h"
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

double clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
}

double normalizedDistance(double distance, double scale) {
    return clamp01(distance / std::max(1.0, scale));
}

uint64_t partOperatorKey(size_t part, OperatorKind kind) {
    return (static_cast<uint64_t>(part) << 8) ^ static_cast<uint64_t>(kind);
}

void incrementSummary(ActiveMoveSummary& summary, OperatorKind kind) {
    switch (kind) {
    case OperatorKind::ContactPacking:
        ++summary.contact;
        break;
    case OperatorKind::Compression:
        ++summary.compression;
        break;
    case OperatorKind::GapFilling:
        ++summary.gap;
        break;
    case OperatorKind::HoleFilling:
        ++summary.hole;
        break;
    case OperatorKind::ConcavityFilling:
        ++summary.concavity;
        break;
    case OperatorKind::SmallPartFiller:
        ++summary.smallPart;
        break;
    case OperatorKind::Swap:
        ++summary.swap;
        break;
    case OperatorKind::EjectionChain:
        ++summary.chain;
        break;
    case OperatorKind::ClusterRepack:
        ++summary.cluster;
        break;
    case OperatorKind::RegionRepack:
        ++summary.region;
        break;
    case OperatorKind::RotationRefinement:
        ++summary.rotation;
        break;
    case OperatorKind::Mirror:
        ++summary.mirror;
        break;
    case OperatorKind::Escape:
        ++summary.escape;
        break;
    case OperatorKind::Frontier:
        ++summary.frontier;
        break;
    }
}

double summaryTotal(const ActiveMoveSummary& summary) {
    return static_cast<double>(
        summary.contact + summary.compression + summary.gap + summary.hole +
        summary.concavity + summary.smallPart + summary.swap + summary.chain +
        summary.cluster + summary.region + summary.rotation + summary.mirror +
        summary.escape + summary.frontier);
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
    const double threshold = noImprovementSteps > 4 ? 0.08 : 0.16;
    switch (kind) {
    case OperatorKind::ContactPacking:
        return part.need.needsContactPacking > threshold ||
            part.nearBoundary || part.inDenseRegion;
    case OperatorKind::Compression:
        return part.need.needsCompression > threshold || part.hasCollision;
    case OperatorKind::GapFilling:
        return part.need.needsGapMove > threshold;
    case OperatorKind::HoleFilling:
        return part.need.needsHoleFilling > threshold || part.candidateForHole;
    case OperatorKind::ConcavityFilling:
        return part.need.needsConcavityFitting > threshold || part.candidateForConcavity;
    case OperatorKind::SmallPartFiller:
        return (1.0 - part.sizeRank) > 0.45 &&
            (std::max({part.need.needsHoleFilling, part.need.needsConcavityFitting, part.need.needsGapMove}) > threshold ||
             part.need.wastedSpaceAround > threshold * 0.65 ||
             part.need.mobilityScore > threshold * 0.80);
    case OperatorKind::Swap:
        return part.need.needsSwap > threshold;
    case OperatorKind::EjectionChain:
        return part.need.needsHoleFilling > threshold || part.need.needsConcavityFitting > threshold || noImprovementSteps > 4;
    case OperatorKind::ClusterRepack:
        return part.inDenseRegion || part.need.blocksOthers > threshold || noImprovementSteps > 6;
    case OperatorKind::RegionRepack:
        return part.need.wastedSpaceAround > threshold || part.need.needsGapMove > threshold;
    case OperatorKind::RotationRefinement:
        return part.need.needsRotation > threshold;
    case OperatorKind::Mirror:
        return part.need.needsMirror > threshold;
    case OperatorKind::Escape:
        return part.need.needsEscape > threshold || noImprovementSteps > 6;
    case OperatorKind::Frontier:
        return part.need.needsGapMove > threshold || part.need.boundaryContribution > threshold;
    }
    return false;
}

double operatorNeedScore(OperatorKind kind, const PartState& part, size_t noImprovementSteps) {
    switch (kind) {
    case OperatorKind::ContactPacking:
        return part.need.needsContactPacking;
    case OperatorKind::Compression:
        return part.need.needsCompression;
    case OperatorKind::GapFilling:
        return part.need.needsGapMove;
    case OperatorKind::HoleFilling:
        return part.need.needsHoleFilling;
    case OperatorKind::ConcavityFilling:
        return part.need.needsConcavityFitting;
    case OperatorKind::SmallPartFiller:
        return (1.0 - part.sizeRank) * std::max({part.need.needsHoleFilling, part.need.needsConcavityFitting, part.need.needsGapMove});
    case OperatorKind::Swap:
        return part.need.needsSwap;
    case OperatorKind::EjectionChain:
        return std::max(part.need.needsHoleFilling, part.need.needsConcavityFitting) + (noImprovementSteps > 4 ? 0.18 : 0.0);
    case OperatorKind::ClusterRepack:
        return part.need.blocksOthers * 0.65 + (part.inDenseRegion ? 0.25 : 0.0);
    case OperatorKind::RegionRepack:
        return part.need.wastedSpaceAround * 0.65 + part.need.needsGapMove * 0.35;
    case OperatorKind::RotationRefinement:
        return part.need.needsRotation;
    case OperatorKind::Mirror:
        return part.need.needsMirror;
    case OperatorKind::Escape:
        return part.need.needsEscape + (noImprovementSteps > 6 ? 0.25 : 0.0);
    case OperatorKind::Frontier:
        return part.need.needsGapMove * 0.55 + part.need.boundaryContribution * 0.35;
    }
    return 0.0;
}

double operatorSchedulerBias(OperatorKind kind, const std::unordered_map<OperatorKind, OperatorStats>& stats) {
    auto it = stats.find(kind);
    if (it == stats.end()) {
        return 1.0;
    }
    return std::max(0.50, std::min(2.25, std::sqrt(it->second.schedulerScore())));
}

size_t moveTaskLimit(const EngineSettings& settings, size_t partCount) {
    size_t limit = settings.performanceProfile == PerformanceProfile::Maximum ? 120u :
        settings.performanceProfile == PerformanceProfile::Balanced ? 72u : 36u;
    if (partCount > 300 && settings.performanceProfile != PerformanceProfile::Maximum) {
        limit = std::min<size_t>(limit, 56u);
    }
    return std::max<size_t>(16u, std::min(limit, partCount * 4u + 8u));
}

std::vector<MoveTask> buildMoveTasks(
    const EngineSettings& settings,
    const std::vector<PartState>& partStates,
    const std::unordered_map<OperatorKind, OperatorStats>& operatorStats,
    const std::unordered_map<uint64_t, double>& partOperatorBias,
    size_t noImprovementSteps) {
    static constexpr std::array<OperatorKind, 14> kinds{
        OperatorKind::ContactPacking,
        OperatorKind::Compression,
        OperatorKind::GapFilling,
        OperatorKind::HoleFilling,
        OperatorKind::ConcavityFilling,
        OperatorKind::SmallPartFiller,
        OperatorKind::Swap,
        OperatorKind::EjectionChain,
        OperatorKind::ClusterRepack,
        OperatorKind::RegionRepack,
        OperatorKind::RotationRefinement,
        OperatorKind::Mirror,
        OperatorKind::Escape,
        OperatorKind::Frontier
    };
    std::vector<MoveTask> tasks;
    tasks.reserve(partStates.size() * 3u);
    for (const PartState& part : partStates) {
        for (OperatorKind kind : kinds) {
            if (!operatorApplies(kind, part, noImprovementSteps)) {
                continue;
            }
            const double need = operatorNeedScore(kind, part, noImprovementSteps);
            if (need <= 0.035) {
                continue;
            }
            double bias = 1.0;
            auto biasIt = partOperatorBias.find(partOperatorKey(part.part, kind));
            if (biasIt != partOperatorBias.end()) {
                bias = biasIt->second;
            }
            const double scheduler = operatorSchedulerBias(kind, operatorStats);
            const double collisionBoost = part.hasCollision ? 8.0 : 0.0;
            const double mixedSizeBoost = (kind == OperatorKind::HoleFilling ||
                                           kind == OperatorKind::ConcavityFilling ||
                                           kind == OperatorKind::SmallPartFiller ||
                                           kind == OperatorKind::GapFilling)
                ? (1.0 - part.sizeRank) * 0.45
                : part.sizeRank * 0.12;
            MoveTask task;
            task.partIndex = part.part;
            task.operatorType = kind;
            task.estimatedGain = need * (1.0 + part.potentialScore);
            double profileBias = 1.0;
            if (settings.performanceProfile != PerformanceProfile::Fast) {
                switch (kind) {
                case OperatorKind::ContactPacking:
                    profileBias = 2.30;
                    break;
                case OperatorKind::HoleFilling:
                case OperatorKind::ConcavityFilling:
                    profileBias = 2.10;
                    break;
                case OperatorKind::Frontier:
                    profileBias = 1.65;
                    break;
                case OperatorKind::RotationRefinement:
                case OperatorKind::Mirror:
                    profileBias = 1.35;
                    break;
                case OperatorKind::Compression:
                    profileBias = 0.55;
                    break;
                default:
                    profileBias = 1.0;
                    break;
                }
            }
            task.priority = collisionBoost +
                part.priority * 0.08 +
                task.estimatedGain * 100.0 * scheduler * bias * profileBias +
                mixedSizeBoost * 45.0;
            tasks.push_back(task);
        }
    }
    std::stable_sort(tasks.begin(), tasks.end(), [](const MoveTask& a, const MoveTask& b) {
        if (std::abs(a.priority - b.priority) > 1e-9) {
            return a.priority > b.priority;
        }
        if (a.partIndex != b.partIndex) {
            return a.partIndex < b.partIndex;
        }
        return static_cast<int>(a.operatorType) < static_cast<int>(b.operatorType);
    });
    const size_t limit = moveTaskLimit(settings, partStates.size());
    if (tasks.size() > limit) {
        tasks.resize(limit);
    }
    return tasks;
}

size_t highPotentialPartCount(const std::vector<PartState>& parts) {
    return static_cast<size_t>(std::count_if(parts.begin(), parts.end(), [](const PartState& part) {
        return part.potentialScore > 0.52 ||
            part.need.needsHoleFilling > 0.45 ||
            part.need.needsConcavityFitting > 0.45 ||
            part.need.needsRotation > 0.45 ||
            part.need.needsGapMove > 0.62;
    }));
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
    const std::vector<FreeSpaceCandidate>& freeSpace,
    const std::vector<FrontierCandidate>& frontiers) {
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
    std::vector<double> footprints(document.parts.size(), 0.0);
    for (size_t i = 0; i < document.parts.size(); ++i) {
        footprints[i] = partFootprint(document.parts[i]);
    }
    std::vector<double> sortedFootprints = footprints;
    std::sort(sortedFootprints.begin(), sortedFootprints.end());
    const double usedArea = std::max(1.0, used.area());
    const double sheetArea = sheetUsableArea(document, settings);
    const double usedScale = std::max(1.0, std::sqrt(usedArea));

    for (size_t i = 0; i < state.poses.size() && i < document.parts.size(); ++i) {
        PartState part;
        part.part = i;
        const AABB bounds = cache.partBounds()[i];
        const double footprint = footprints[i];
        const Vec2 center = bounds.center();
        const auto rankIt = std::upper_bound(sortedFootprints.begin(), sortedFootprints.end(), footprint);
        part.sizeRank = sortedFootprints.empty()
            ? 0.0
            : static_cast<double>(std::distance(sortedFootprints.begin(), rankIt)) / static_cast<double>(sortedFootprints.size());
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
        double bestRotationGain = 0.0;
        if (settings.allowRotation && settings.rotationMode != RotationMode::None) {
            PoseSampler sampler;
            const double currentArea = std::max(1.0, bounds.area());
            size_t checked = 0;
            for (double angle : sampler.coarseRotationSamples(settings)) {
                Pose candidatePose = state.poses[i];
                candidatePose.angleRadians = angle;
                const AABB rotated = transformedBounds(document.parts[i], candidatePose);
                bestRotationGain = std::max(bestRotationGain, (currentArea - rotated.area()) / currentArea);
                if (++checked >= 8) {
                    break;
                }
            }
        }
        part.rotationSensitive = settings.allowRotation && (aspect > 1.12 || bestRotationGain > 0.035);
        part.mirrorSensitive = settings.allowMirroring && (aspect > 1.05 || document.parts[i].rings.size() > 1);
        part.candidateForHole = footprint <= avgArea * 0.95;
        part.candidateForConcavity = footprint <= avgArea * 1.20;
        double holeScore = 0.0;
        double concavityScore = 0.0;
        double gapScore = 0.0;
        size_t fittingHoleCount = 0;
        size_t fittingConcavityCount = 0;
        for (const FreeSpaceCandidate& candidate : freeSpace) {
            if (candidate.sourcePart == i) {
                continue;
            }
            const double featureArea = candidate.featureBounds.isValid() ? candidate.featureBounds.area() : 0.0;
            const double centerDistance = (candidate.anchor - center).length();
            const double distanceBoost = 1.0 - normalizedDistance(centerDistance, usedScale);
            if (candidate.kind == FreeSpaceCandidateKind::PartHole && featureArea >= footprint * 1.02) {
                part.candidateForHole = true;
                ++fittingHoleCount;
                holeScore = std::max(holeScore, 0.45 + 0.35 * distanceBoost + 0.20 * (1.0 - part.sizeRank));
            }
            if (candidate.kind == FreeSpaceCandidateKind::Concavity && featureArea >= footprint * 0.55) {
                part.candidateForConcavity = true;
                ++fittingConcavityCount;
                concavityScore = std::max(concavityScore, 0.35 + 0.35 * distanceBoost + 0.20 * (1.0 - part.sizeRank));
            }
            if ((candidate.kind == FreeSpaceCandidateKind::UsedBoundsGap ||
                 candidate.kind == FreeSpaceCandidateKind::SheetBoundary ||
                 candidate.kind == FreeSpaceCandidateKind::SheetCorner) &&
                (featureArea <= 0.0 || featureArea >= footprint * 0.65)) {
                gapScore = std::max(gapScore, 0.20 + 0.30 * distanceBoost + 0.35 * (1.0 - part.sizeRank));
            }
        }
        for (const FrontierCandidate& frontier : frontiers) {
            if (frontier.sourcePart == i) {
                continue;
            }
            const double featureArea = frontier.featureBounds.isValid() ? frontier.featureBounds.area() : 0.0;
            if (featureArea <= 0.0 || featureArea >= footprint * 0.50) {
                const double distanceBoost = 1.0 - normalizedDistance((frontier.anchor - center).length(), usedScale);
                gapScore = std::max(gapScore, 0.25 + 0.35 * distanceBoost + 0.25 * (1.0 - part.sizeRank));
            }
        }

        double boundaryContribution = 0.0;
        if (used.isValid()) {
            if (std::abs(bounds.min.x - used.min.x) <= boundaryEps || std::abs(bounds.max.x - used.max.x) <= boundaryEps) {
                boundaryContribution += bounds.width() / std::max(1.0, used.width());
            }
            if (std::abs(bounds.min.y - used.min.y) <= boundaryEps || std::abs(bounds.max.y - used.max.y) <= boundaryEps) {
                boundaryContribution += bounds.height() / std::max(1.0, used.height());
            }
        }
        boundaryContribution = clamp01(boundaryContribution);
        const double neighborDensity = clamp01(static_cast<double>(part.neighborCount) / static_cast<double>(document.parts.size() > 120 ? 12u : 8u));
        const double freeAround = clamp01(1.0 - neighborDensity);
        const double footprintShare = clamp01(footprint / sheetArea * static_cast<double>(document.parts.size()));
        const double wastedSpace = clamp01((freeAround * 0.55) + (boundaryContribution * 0.30) + (gapScore * 0.35));
        part.need.boundaryContribution = boundaryContribution;
        part.need.wastedSpaceAround = wastedSpace;
        part.need.mobilityScore = clamp01(freeAround * 0.70 + gapScore * 0.30);
        part.need.blocksOthers = clamp01(neighborDensity * 0.45 + boundaryContribution * 0.35 + footprintShare * 0.20);
        part.need.needsCompression = clamp01(boundaryContribution * 0.65 + wastedSpace * 0.25 + (part.hasCollision ? 1.0 : 0.0));
        part.need.needsRotation = clamp01(bestRotationGain * 3.5 + (part.rotationSensitive ? 0.20 : 0.0) + boundaryContribution * 0.10);
        part.need.needsMirror = clamp01((part.mirrorSensitive ? 0.25 : 0.0) + holeScore * 0.25 + concavityScore * 0.20);
        part.need.needsHoleFilling = clamp01(holeScore + static_cast<double>(fittingHoleCount > 0 ? 1 : 0) * 0.20);
        part.need.needsConcavityFitting = clamp01(concavityScore + static_cast<double>(fittingConcavityCount > 0 ? 1 : 0) * 0.15);
        part.need.needsContactPacking = clamp01(
            part.need.boundaryContribution * 0.25 +
            part.need.wastedSpaceAround * 0.20 +
            part.need.mobilityScore * 0.20 +
            holeScore * 0.20 +
            concavityScore * 0.15 +
            (part.inDenseRegion ? 0.18 : 0.0));
        part.need.needsGapMove = clamp01(gapScore + wastedSpace * 0.35 + freeAround * 0.15);
        part.need.needsSwap = clamp01(part.need.blocksOthers * 0.55 + neighborDensity * 0.30 + boundaryContribution * 0.15);
        part.need.needsEscape = clamp01((part.hasCollision ? 0.40 : 0.0) + wastedSpace * 0.20 + neighborDensity * 0.15);
        part.potentialScore = std::max({
            part.need.needsCompression,
            part.need.needsRotation,
            part.need.needsMirror,
            part.need.needsHoleFilling,
            part.need.needsConcavityFitting,
            part.need.needsContactPacking,
            part.need.needsGapMove,
            part.need.needsSwap,
            part.need.needsEscape,
            part.need.blocksOthers,
            part.need.wastedSpaceAround,
            part.need.boundaryContribution
        });
        part.priority =
            (part.hasCollision ? 10000.0 : 0.0) +
            part.potentialScore * 180.0 +
            part.need.boundaryContribution * 120.0 +
            part.need.blocksOthers * 90.0 +
            (1.0 - part.sizeRank) * (part.need.needsHoleFilling + part.need.needsGapMove) * 80.0 +
            footprint / sheetArea * 40.0;
        parts.push_back(part);
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
    incrementSummary(stats.acceptedMoveSummary, kind);
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
    incrementSummary(stats.activeMoveSummary, kind);
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
    std::unordered_map<uint64_t, double> partOperatorBias;
    SolverStrategy currentStrategy = SolverStrategy::AdaptiveSearch;
    uint64_t versionId = 1;

    auto publish = [&](const ActiveMoveSummary& activeMoves, bool layoutChanged, const CandidateMove& move, bool bestUpdated) {
        if (callback) {
            AdaptiveProgressEvent event;
            event.strategy = currentStrategy;
            event.current = current;
            event.best = best;
            event.stats = stats;
            event.activeMoves = activeMoves;
            event.versionId = versionId;
            event.layoutChanged = layoutChanged;
            event.lastMovedPart = move.part;
            event.lastMoveStrategy = strategyForOperator(move.source);
            event.bestUpdated = bestUpdated;
            callback(event);
        }
    };
    auto timeExpired = [&]() {
        return elapsedSeconds(started) >= std::max(0.05, safetyLimit - safetyGuard);
    };

    for (int step = 0; step < maxStepCount && !stopRequested.load() && !timeExpired(); ++step) {
        LayoutEvalCache cache;
        cache.rebuild(document, settings, current, &attemptPenalties, &globalPenalties, 0.10);

        FreeSpaceAnalyzer freeAnalyzer;
        FrontierAnalyzer frontierAnalyzer;
        const std::vector<FreeSpaceCandidate> freeSpace = freeAnalyzer.analyze(document, settings, current);
        const std::vector<FrontierCandidate> frontiers = frontierAnalyzer.analyze(document, settings, current);
        EmptySpaceAnalyzer emptyAnalyzer;
        const EmptySpaceMap emptyMap = emptyAnalyzer.analyze(document, settings, current);
        std::vector<PartState> partStates = analyzeParts(document, settings, current, cache, freeSpace, frontiers);
        std::vector<MoveTask> tasks = buildMoveTasks(
            settings,
            partStates,
            operatorStats_,
            partOperatorBias,
            static_cast<size_t>(std::max(0, convergence.noImprovementSteps)));
        if (settings.performanceProfile != PerformanceProfile::Fast && step % 10 == 0) {
            std::vector<const PartState*> smallParts;
            smallParts.reserve(partStates.size());
            for (const PartState& part : partStates) {
                if ((1.0 - part.sizeRank) > 0.45) {
                    smallParts.push_back(&part);
                }
            }
            std::stable_sort(smallParts.begin(), smallParts.end(), [](const PartState* a, const PartState* b) {
                const double scoreA = (1.0 - a->sizeRank) * 80.0 +
                    a->need.needsGapMove * 55.0 +
                    a->need.needsHoleFilling * 45.0 +
                    a->need.needsConcavityFitting * 45.0 +
                    a->need.mobilityScore * 20.0;
                const double scoreB = (1.0 - b->sizeRank) * 80.0 +
                    b->need.needsGapMove * 55.0 +
                    b->need.needsHoleFilling * 45.0 +
                    b->need.needsConcavityFitting * 45.0 +
                    b->need.mobilityScore * 20.0;
                return scoreA > scoreB;
            });
            size_t forced = 0;
            for (const PartState* part : smallParts) {
                if (forced >= 12u) {
                    break;
                }
                const bool exists = std::any_of(tasks.begin(), tasks.end(), [&](const MoveTask& task) {
                    return task.partIndex == part->part && task.operatorType == OperatorKind::SmallPartFiller;
                });
                if (exists) {
                    continue;
                }
                MoveTask task;
                task.partIndex = part->part;
                task.operatorType = OperatorKind::SmallPartFiller;
                task.estimatedGain = std::max({part->need.needsGapMove, part->need.needsHoleFilling, part->need.needsConcavityFitting, 0.20});
                task.priority = 420.0 + task.estimatedGain * 140.0 + (1.0 - part->sizeRank) * 70.0;
                tasks.push_back(task);
                ++forced;
            }
            std::stable_sort(tasks.begin(), tasks.end(), [](const MoveTask& a, const MoveTask& b) {
                if (std::abs(a.priority - b.priority) > 1e-9) {
                    return a.priority > b.priority;
                }
                if (a.partIndex != b.partIndex) {
                    return a.partIndex < b.partIndex;
                }
                return static_cast<int>(a.operatorType) < static_cast<int>(b.operatorType);
            });
            const size_t limit = moveTaskLimit(settings, partStates.size()) + 12u;
            if (tasks.size() > limit) {
                tasks.resize(limit);
            }
        }
        convergence.highPotentialParts = highPotentialPartCount(partStates);
        convergence.promisingTasks = static_cast<size_t>(std::count_if(tasks.begin(), tasks.end(), [](const MoveTask& task) {
            return task.priority > 70.0 || task.estimatedGain > 0.55;
        }));
        const double avgSmallArea = std::max(1.0, document.totalPartArea() / std::max<size_t>(1, document.parts.size()) * 0.35);
        const LayoutShapeMetrics shapeMetrics = computeLayoutShapeMetrics(document, settings, emptyMap.usedBounds);
        convergence.freeSpacePotential = emptyMap.totalEmptyArea / sheetUsableArea(document, settings);
        convergence.largeEmptyRegionArea = emptyMap.largestRegionArea;
        convergence.fillableGapCount = emptyMap.fillableRegionCount(avgSmallArea);
        convergence.movablePartCount = static_cast<size_t>(std::count_if(partStates.begin(), partStates.end(), [](const PartState& part) {
            return part.need.mobilityScore > 0.22 || part.need.needsGapMove > 0.35 || part.need.wastedSpaceAround > 0.28;
        }));
        convergence.contactOpportunityCount = frontiers.size() + freeSpace.size();
        convergence.unusedHoleCapacity = static_cast<size_t>(std::count_if(freeSpace.begin(), freeSpace.end(), [](const FreeSpaceCandidate& candidate) {
            return candidate.kind == FreeSpaceCandidateKind::PartHole;
        }));
        stats.emptySpaceArea = emptyMap.totalEmptyArea;
        stats.largestEmptyRegionArea = emptyMap.largestRegionArea;
        stats.fillableGapCount = convergence.fillableGapCount;
        stats.contactCount = static_cast<size_t>(std::llround(std::max(0.0, current.contactReward)));
        stats.towerScore = shapeMetrics.towerScore;
        stats.layoutSpreadScore = shapeMetrics.layoutSpreadScore;
        stats.unusedSheetRegionScore = shapeMetrics.unusedSheetRegionScore;

        OperatorContext context{document, settings, cache, freeSpace, frontiers, partStates, static_cast<size_t>(step)};
        std::vector<std::unique_ptr<IOperator>> operators = makeOperators(context);
        auto findOperator = [&](OperatorKind kind) -> IOperator* {
            for (const std::unique_ptr<IOperator>& op : operators) {
                if (op->kind() == kind) {
                    return op.get();
                }
            }
            return nullptr;
        };

        CandidateMove bestMove;
        LayoutState bestMoveState;
        bool foundMove = false;
        bool acceptedWorse = false;
        double bestCandidateScore = std::numeric_limits<double>::max();
        ActiveMoveSummary stepActiveMoves;

        for (const MoveTask& task : tasks) {
            if (timeExpired() || stopRequested.load()) {
                break;
            }
            if (task.partIndex >= partStates.size()) {
                continue;
            }
            IOperator* op = findOperator(task.operatorType);
            if (!op) {
                continue;
            }
            const PartState& target = partStates[task.partIndex];
            if (!operatorApplies(task.operatorType, target, static_cast<size_t>(std::max(0, convergence.noImprovementSteps)))) {
                continue;
            }
            incrementSummary(stepActiveMoves, task.operatorType);
            std::vector<CandidateMove> candidates;
            op->generateCandidates(current, target, candidates);
            const size_t baseBudget = operatorBudget(settings, operatorStats_[task.operatorType]);
            const size_t taskBudget = std::max<size_t>(2, std::min<size_t>(baseBudget + static_cast<size_t>(task.estimatedGain * 4.0), 24u));
            if (candidates.size() > taskBudget) {
                candidates.resize(taskBudget);
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
                    ++stats.rejectedWorseMoves;
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
                const double verifiedImprovement = current.totalScore - verified.totalScore;
                const bool improvesCurrent = verifiedImprovement > 1e-9;
                if ((improvesCurrent && verified.totalScore + 1e-9 < bestCandidateScore) || (!foundMove && allowWorse)) {
                    bestCandidateScore = verified.totalScore;
                    bestMoveState = std::move(verified);
                    bestMove = std::move(candidate);
                    foundMove = true;
                    acceptedWorse = bestMoveState.totalScore > current.totalScore + 1e-9;
                    currentStrategy = strategyForOperator(bestMove.source);
                }
            }
        }

        bool bestUpdated = false;
        if (foundMove) {
            const double previousScore = current.totalScore;
            current = std::move(bestMoveState);
            const double improvement = std::max(0.0, previousScore - current.totalScore);
            ++operatorStats_[bestMove.source].accepted;
            operatorStats_[bestMove.source].totalImprovement += improvement / std::max(1.0, std::abs(previousScore));
            operatorStats_[bestMove.source].lastImprovementTime = elapsedSeconds(started);
            recordAccepted(bestMove.source, stats);
            const uint64_t acceptedKey = partOperatorKey(bestMove.part, bestMove.source);
            const double currentBias = partOperatorBias.count(acceptedKey) != 0 ? partOperatorBias[acceptedKey] : 1.0;
            partOperatorBias[acceptedKey] = std::min(2.5, currentBias + 0.25);
            if (acceptedWorse) {
                ++stats.acceptedWorseMoves;
            }
            if (current.valid() && current.totalScore + 1e-9 < best.totalScore) {
                best = current;
                ++stats.bestUpdates;
                convergence.noImprovementSteps = 0;
                convergence.lastBestScore = best.totalScore;
                bestUpdated = true;
            } else {
                ++convergence.noImprovementSteps;
            }
        } else {
            ++convergence.noImprovementSteps;
            const size_t penaltyLimit = std::min<size_t>(tasks.size(), 12u);
            for (size_t i = 0; i < penaltyLimit; ++i) {
                const uint64_t key = partOperatorKey(tasks[i].partIndex, tasks[i].operatorType);
                const double currentBias = partOperatorBias.count(key) != 0 ? partOperatorBias[key] : 1.0;
                partOperatorBias[key] = std::max(0.35, currentBias * 0.88);
            }
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

        if (foundMove) {
            ++versionId;
            publish(stepActiveMoves, true, bestMove, bestUpdated);
        }

        const bool utilizationPlateau = best.utilization <= current.utilization + 1e-6 &&
            convergence.noImprovementSteps > stagnantThreshold;
        const bool repeated = convergence.repeatedLayouts >= 4;
        const bool lowAcceptance = convergence.lowAcceptanceSteps > stagnantThreshold / 2;
        const bool exploredThisStep = summaryTotal(stepActiveMoves) > 0.0;
        double utilizationTarget = 0.0;
        if (settings.performanceProfile == PerformanceProfile::Maximum) {
            utilizationTarget = document.parts.size() >= 400 ? 0.72 :
                document.parts.size() >= 80 ? 0.65 : 0.0;
        } else if (settings.performanceProfile == PerformanceProfile::Balanced && document.parts.size() >= 80) {
            utilizationTarget = 0.65;
        }
        const bool belowIndustrialTarget = utilizationTarget > 0.0 && best.utilization + 1e-6 < utilizationTarget;
        const bool largeEmptyRegion = convergence.largeEmptyRegionArea > avgSmallArea * 3.0 ||
            convergence.freeSpacePotential > 0.10;
        const bool fillableGapsRemain = convergence.fillableGapCount > std::max<size_t>(1u, document.parts.size() / 160u);
        const bool contactOpportunitiesRemain = convergence.contactOpportunityCount > std::max<size_t>(8u, document.parts.size() / 8u);
        const bool movablePartsRemain = convergence.movablePartCount > std::max<size_t>(4u, document.parts.size() / 80u);
        const bool towerNeedsRepack = shapeMetrics.towerScore > 0.20 && convergence.fillableGapCount > 0;
        const bool qualityPotential =
            belowIndustrialTarget &&
            (largeEmptyRegion || fillableGapsRemain || contactOpportunitiesRemain || movablePartsRemain || towerNeedsRepack || convergence.unusedHoleCapacity > 0);
        const bool stillHasPotential =
            convergence.highPotentialParts > std::max<size_t>(1u, document.parts.size() / 140u) ||
            convergence.promisingTasks > std::max<size_t>(2u, tasks.size() / 6u) ||
            qualityPotential;
        const bool stronglyStalled =
            convergence.noImprovementSteps > stagnantThreshold * 3 &&
            lowAcceptance &&
            repeated &&
            exploredThisStep;
        if ((!stillHasPotential &&
             (convergence.noImprovementSteps > stagnantThreshold ||
              (utilizationPlateau && lowAcceptance) ||
              repeated)) ||
            (stronglyStalled && !qualityPotential)) {
            break;
        }
    }

    LayoutState finalBest = scorer.evaluate(document, settings, best.poses, &attemptPenalties, &globalPenalties, 0.10);
    return finalBest.valid() ? finalBest : best;
}

} // namespace nest
