#pragma once

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

struct EngineSettings {
    double sheetWidth = 1000.0;
    double sheetHeight = 600.0;
    double partSpacing = 5.0;
    double margin = 10.0;
    bool allowRotation = true;
    RotationMode rotationMode = RotationMode::RightAngles;
    double rotationStepDegrees = 1.0;
    bool allowMirroring = false;
    QualityMode qualityMode = QualityMode::Balanced;
    double timeLimitSeconds = 30.0;
    int cpuThreadCount = 1;
    bool useGpuFutureFlag = false;
    int livePreviewIntervalMs = 75;
    double collisionTolerance = 0.01;
    double curveFlattenTolerance = 0.35;
};

} // namespace nest
