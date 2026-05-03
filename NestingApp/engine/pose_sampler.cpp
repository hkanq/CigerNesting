#include "engine/pose_sampler.h"

#include "core/math_utils.h"
#include <algorithm>
#include <cmath>
#include <random>

namespace nest {
namespace {

constexpr double kMaxCoarseRotationSamples = 720.0;

double safeFixedStepDegrees(double requestedStepDegrees) {
    const double requested = std::max(0.001, requestedStepDegrees);
    const double sampleCount = std::ceil(360.0 / requested);
    if (sampleCount > kMaxCoarseRotationSamples) {
        return 360.0 / kMaxCoarseRotationSamples;
    }
    return requested;
}

} // namespace

std::vector<double> PoseSampler::coarseRotationSamples(const EngineSettings& settings) const {
    if (!settings.allowRotation || settings.rotationMode == RotationMode::None) {
        return {0.0};
    }

    double step = 90.0;
    if (settings.rotationMode == RotationMode::FortyFiveDegrees) {
        step = 45.0;
    } else if (settings.rotationMode == RotationMode::FixedStep) {
        step = safeFixedStepDegrees(settings.rotationStepDegrees);
    } else if (settings.rotationMode == RotationMode::ContinuousRefine) {
        step = 90.0;
    }

    std::vector<double> samples;
    for (double degrees = 0.0; degrees < 360.0 - 1e-9; degrees += step) {
        samples.push_back(degreesToRadians(degrees));
    }
    if (samples.empty()) {
        samples.push_back(0.0);
    }
    return samples;
}

std::vector<double> PoseSampler::refinementRotationSamples(double centerRadians, double stepDegrees, int radius) const {
    const double step = degreesToRadians(std::max(0.001, stepDegrees));
    std::vector<double> samples;
    for (int i = -radius; i <= radius; ++i) {
        samples.push_back(centerRadians + static_cast<double>(i) * step);
    }
    return samples;
}

std::vector<bool> PoseSampler::mirrorSamples(const EngineSettings& settings) const {
    return settings.allowMirroring ? std::vector<bool>{false, true} : std::vector<bool>{false};
}

std::vector<Pose> PoseSampler::moveCandidates(const Document& document, const EngineSettings& settings, const std::vector<Pose>& poses, size_t partIndex, unsigned int seed, int iteration) const {
    std::vector<Pose> candidates;
    if (partIndex >= poses.size() || partIndex >= document.parts.size()) {
        return candidates;
    }

    const Pose base = poses[partIndex];
    candidates.push_back(base);

    const double small = std::max(1.0, settings.partSpacing * 0.5 + 1.0);
    const double medium = std::max(4.0, settings.partSpacing * 2.0 + 4.0 + static_cast<double>(iteration % 4));
    const double large = std::max(12.0, std::min(settings.sheetWidth, settings.sheetHeight) * 0.08);
    const double steps[] = {small, medium, large};

    for (const double step : steps) {
        const Vec2 offsets[] = {
            {step, 0.0}, {-step, 0.0}, {0.0, step}, {0.0, -step},
            {step, step}, {-step, step}, {step, -step}, {-step, -step}
        };
        for (const Vec2& offset : offsets) {
            Pose pose = base;
            pose.x += offset.x;
            pose.y += offset.y;
            candidates.push_back(pose);
        }
    }

    const AABB currentBounds = transformedBounds(document.parts[partIndex], base);
    const double left = settings.margin;
    const double right = settings.sheetWidth - settings.margin;
    const double bottom = settings.margin;
    const double top = settings.sheetHeight - settings.margin;
    const Vec2 snaps[] = {
        {left - currentBounds.min.x, 0.0},
        {right - currentBounds.max.x, 0.0},
        {0.0, bottom - currentBounds.min.y},
        {0.0, top - currentBounds.max.y}
    };
    for (const Vec2& delta : snaps) {
        Pose pose = base;
        pose.x += delta.x;
        pose.y += delta.y;
        candidates.push_back(pose);
    }

    auto addBoundsContactCandidates = [&](const AABB& target, double clearance) {
        if (!target.isValid()) {
            return;
        }
        const Vec2 contactSnaps[] = {
            {target.max.x + clearance - currentBounds.min.x, 0.0},
            {target.min.x - clearance - currentBounds.max.x, 0.0},
            {0.0, target.max.y + clearance - currentBounds.min.y},
            {0.0, target.min.y - clearance - currentBounds.max.y},
            {target.min.x - currentBounds.min.x, target.min.y - currentBounds.min.y},
            {target.max.x - currentBounds.max.x, target.max.y - currentBounds.max.y}
        };
        for (const Vec2& delta : contactSnaps) {
            Pose pose = base;
            pose.x += delta.x;
            pose.y += delta.y;
            candidates.push_back(pose);
        }
    };

    for (size_t i = 0; i < document.parts.size() && i < poses.size(); ++i) {
        if (i == partIndex) {
            continue;
        }
        const AABB other = transformedBounds(document.parts[i], poses[i]);
        addBoundsContactCandidates(other, settings.partSpacing);
    }

    addBoundsContactCandidates(AABB::fromMinMax(
        {document.sheet.origin.x + settings.margin, document.sheet.origin.y + settings.margin},
        {document.sheet.origin.x + document.sheet.width - settings.margin, document.sheet.origin.y + document.sheet.height - settings.margin}), 0.0);

    if (document.sheet.hasCustomProfile()) {
        auto ringBounds = [](const Ring& ring) {
            AABB box;
            for (const Vec2& point : ring.points) {
                box.include(point);
            }
            return box;
        };
        addBoundsContactCandidates(ringBounds(document.sheet.profile().outerContour), 0.0);
        for (const Ring& hole : document.sheet.profile().holes) {
            addBoundsContactCandidates(ringBounds(hole), settings.partSpacing);
        }
        for (const Ring& zone : document.sheet.profile().forbiddenZones) {
            addBoundsContactCandidates(ringBounds(zone), settings.partSpacing);
        }
    }

    if (settings.allowRotation && settings.rotationMode != RotationMode::None) {
        std::vector<double> rotations = coarseRotationSamples(settings);
        const double refineStep = degreesToRadians(std::max(0.001, settings.rotationStepDegrees));
        rotations.push_back(base.angleRadians + refineStep);
        rotations.push_back(base.angleRadians - refineStep);
        for (double angle : rotations) {
            Pose pose = base;
            pose.angleRadians = angle;
            candidates.push_back(pose);
        }
    }

    if (settings.allowMirroring) {
        Pose pose = base;
        pose.mirrored = !pose.mirrored;
        candidates.push_back(pose);
    }

    std::mt19937 rng(seed + static_cast<unsigned int>(partIndex * 7919u) + static_cast<unsigned int>(iteration * 104729u));
    std::uniform_real_distribution<double> dx(-large, large);
    for (int i = 0; i < 8; ++i) {
        Pose pose = base;
        pose.x += dx(rng);
        pose.y += dx(rng);
        candidates.push_back(pose);
    }

    return candidates;
}

} // namespace nest
