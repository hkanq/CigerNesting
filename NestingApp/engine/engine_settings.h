#pragma once

#include <cstdint>

namespace nest {

enum class RotationMode {
    None,
    RightAngles,
    FortyFiveDegrees,
    FixedStep,
    ContinuousRefine
};

enum class QualityMode {
    Fast,
    Balanced,
    MaxQuality
};

enum class PerformanceProfile {
    Fast,
    Balanced,
    Maximum
};

enum class PlacementStrategy {
    BottomLeft,
    TopLeft,
    BottomRight,
    TopRight,
    LeftToRight,
    RightToLeft,
    TopToBottom,
    BottomToTop,
    CenterOut,
    OutsideIn,
    UserPoints
};

struct EngineSettings {
    double sheetWidth = 1000.0;
    double sheetHeight = 600.0;
    double partSpacing = 5.0;
    double margin = 10.0;
    PlacementStrategy placementStrategy = PlacementStrategy::BottomLeft;
    bool allowRotation = true;
    RotationMode rotationMode = RotationMode::ContinuousRefine;
    double rotationStepDegrees = 0.001;
    bool allowMirroring = false;
    QualityMode qualityMode = QualityMode::MaxQuality;
    PerformanceProfile performanceProfile = PerformanceProfile::Maximum;
    double timeLimitSeconds = 0.0;
    int cpuThreadCount = 0;
    bool useGpuFutureFlag = false;
    int livePreviewIntervalMs = 75;
    double collisionTolerance = 0.01;
    double curveFlattenTolerance = 0.35;
    uint32_t randomSeed = 1u;
    bool deterministic = false;
};

} // namespace nest
