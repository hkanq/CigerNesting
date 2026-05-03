#pragma once

namespace nest {

enum class Language {
    Auto,
    Turkish,
    English
};

enum class TextId {
    AppTitle,
    FileOpen,
    Save,
    Start,
    Stop,
    CorelConnection,
    ExportToCorel,
    Settings,
    NestingSettings,
    SheetWidth,
    SheetHeight,
    PartSpacing,
    Margin,
    PlacementStart,
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
    UserPoints,
    RotationEnabled,
    RotationMode,
    AnglePrecision,
    MirroringEnabled,
    QualityMode,
    TimeLimitSeconds,
    ThreadCount,
    Phase,
    Collision,
    Utilization,
    OpenFilePrompt,
    ImportFailed,
    ImportSuccess,
    OpenFileFirst,
    CorelBridgeReady,
    SaveReady,
    Idle,
    PrepareGeometry,
    InitialPlacement,
    Exploration,
    CollisionResolution,
    Compression,
    UltraRefinement,
    FinalValidation,
    Done,
    Stopped,
    Fast,
    Balanced,
    MaxQuality,
    None,
    RightAngles,
    FortyFiveDegrees,
    FixedStep,
    ContinuousRefine,
    FileFilterSupported,
    FileFilterSvg,
    FileFilterDxf,
    FileFilterPlt,
    FileFilterAll,
    FilePanel,
    PartCount,
    NoFileLoaded,
    OpenFileHint,
    PreviewPrompt,
    CorelExportReady,
    Count
};

class Localization {
public:
    static Localization& instance();

    void setLanguage(Language language);
    Language currentLanguage() const;
    Language detectSystemLanguage();
    const wchar_t* text(TextId id) const;

private:
    Localization();

    Language requestedLanguage_ = Language::Auto;
    Language activeLanguage_ = Language::English;
};

} // namespace nest
