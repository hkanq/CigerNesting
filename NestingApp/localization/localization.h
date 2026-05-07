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
    FileMenu,
    Save,
    Start,
    Stop,
    CorelConnection,
    Integrations,
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
    ThreadMaximum,
    ThreadCore,
    Phase,
    CurrentStrategy,
    ActiveMoves,
    LastMove,
    BestUpdate,
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
    ContactPacking,
    GapFilling,
    HoleFilling,
    ConcavityFilling,
    SmallPartFiller,
    Rearrangement,
    Swap,
    EjectionChain,
    ClusterRepack,
    RegionRepack,
    Escape,
    Frontier,
    Mirror,
    UltraRefinement,
    FinalValidation,
    Done,
    NoValidLayout,
    Failed,
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
