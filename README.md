# CigerNesting

Native Windows C++20 nesting application prototype with a modular geometry/import/engine/UI architecture.

## Requirements

- Windows 10 or Windows 11
- Visual Studio Build Tools 2022 or Visual Studio 2022
- Desktop development with C++ workload
- MSVC v143 toolset
- Windows 10/11 SDK
- CMake tools for Windows

No third-party geometry, UI, import, or packaging library is required.

## Build

Recommended Release build:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

If `cmake` is not on `PATH`, use the bundled build helper. It locates Visual Studio Build Tools via `vswhere` and uses the CMake bundled with Visual Studio:

```bat
build_msvc.bat Release
```

Debug build:

```bat
build_msvc.bat Debug
```

## Output

Release executable:

```text
build\NestingApp\Release\CigerNesting.exe
```

Debug executable:

```text
build\NestingApp\Debug\CigerNesting.exe
```

## Smoke Test

1. Run `build\NestingApp\Release\CigerNesting.exe`.
2. Use `Dosya Aç` / `Open File`.
3. Load `samples\simple_polygons.svg` or `samples\basic_shapes.svg`.
4. Choose a `Dizim Başlangıcı` / `Placement Start` strategy from the right panel.
5. Press `Başlat` / `Start`.
6. Verify that the progress bar and phase text update and the canvas layout changes without freezing.

Additional sample files:

```text
samples\simple_plt.plt
samples\simple_dxf.dxf
```

## Placement Strategy

The initial placement stage is no longer hard-wired to a single left-to-right row pass. `EngineSettings::placementStrategy` supports:

- `BottomLeft`, `TopLeft`, `BottomRight`, `TopRight`
- `LeftToRight`, `RightToLeft`
- `TopToBottom`, `BottomToTop`
- `CenterOut`, `OutsideIn`
- `UserPoints`

The current solver still uses a deliberately simple first placement strategy, but the selected direction now flows from the Win32 settings panel into the engine. `CenterOut` and `OutsideIn` use anchor-based starts; `UserPoints` falls back to `BottomLeft` until user-defined anchors are provided.

## Custom Sheet Foundation

`Sheet` now carries a `SheetProfile` model with an outer contour, holes, and forbidden zones while preserving the rectangular `width` / `height` path. This is the foundation for future stock contours such as oval sheets, offcut/hurda boundaries, rounded plates, holes, and no-go zones.

Future sheet import paths are expected to support:

- selecting a stock-area contour from a file
- marking an SVG/DXF contour as the sheet boundary
- receiving sheet contour and parts separately through the Corel bridge

## User Placement Points

`Sheet` includes user placement anchor storage via `addUserPlacementPoint`, `clearUserPlacementPoints`, and `getUserPlacementPoints`. Canvas interaction is not enabled yet, but the engine can already consume these anchors when `PlacementStrategy::UserPoints` is selected.

## Icon

The executable includes a native Win32 icon resource:

```text
NestingApp\resources\CigerNesting.rc
NestingApp\resources\resource.h
NestingApp\resources\app_icon.ico
```

The icon is compiled into `CigerNesting.exe` and loaded for the main window/taskbar class.

## Geometry Validation Layer

The first production-oriented contour validation layer is active:

- world-space transformed rings are produced through `geometry/transformed_shape.*`
- segment intersection handles proper crossing, endpoint touching, collinear overlap, and tolerance
- point-in-ring returns `Outside`, `Inside`, or `OnBoundary`
- part collision is hole-aware: solid area is treated as outer contours minus holes
- parts may occupy another part's hole as long as they do not touch/intersect solid material
- sheet containment validates rectangular sheets, custom sheet outer contours, sheet holes, and forbidden zones
- custom sheet containment checks boundary crossings and multiple samples along each solid edge, so concave sheet cases are no longer limited to vertex/midpoint validation
- clearance has a central boundary-distance API through `geometry/clearance.*`
- part spacing is measured between real part boundaries, including outer and hole rings
- sheet margin is measured from part boundaries to sheet outer, sheet hole, and forbidden-zone boundaries

Collision and clearance are intentionally separate. A small part can fit inside a donut or B-like hole without collision, but it still must keep the requested clearance from the hole boundary. The current clearance implementation is segment-distance based and uses ring AABB pruning and early invalid exits. It intentionally does not yet perform true polygon offsetting, but the interface is designed so a future offset engine can replace the internals without changing solver/UI boundaries.

Geometry smoke test target:

```powershell
build\NestingApp\Release\CigerNestingGeometryCollisionSmoke.exe
```

Clearance smoke test target:

```powershell
build\NestingApp\Release\CigerNestingClearanceSmoke.exe
```

## Continuous Solver V1

The solver is no longer limited to a single demo row placement pass. `NestingEngine` now drives a multi-start collision-driven solver:

- `LayoutState` stores current poses, compactness, collision count, invalid part count, penalties, and total score
- `LayoutScore` evaluates contour collision, spacing, sheet validity, used area, and utilization
- `PenaltySystem` separates per-attempt pair weights from optional low-weight global bias
- `PoseSampler` generates translation, sheet contact, neighbor-edge contact, hole/forbidden-zone contact, rotation, mirror, and random jump candidates
- `GuidedLocalSearch` tries candidate moves for colliding parts and accepts score-improving moves
- `OverlapResolver` runs guided collision resolution while allowing temporarily invalid/overlapping layouts during exploration
- `Compression` searches for safe movement with large-to-small step refinement and only accepts score-improving valid moves
- `MultiStartSolver` runs attempts in parallel through the internal worker pool, cycling placement strategies, seeds, and part orderings within `timeLimitSeconds`, while preserving best-so-far
- `UltraRefinement` refines the best layout locally with angle ladders, micro-translation correction, optional mirroring, and strict contour validation

Solver quality smoke test target:

```powershell
build\NestingApp\Release\CigerNestingSolverQualitySmoke.exe <repo-root>
```

Ultra refinement smoke test target:

```powershell
build\NestingApp\Release\CigerNestingUltraRefinementSmoke.exe
```

## 0.001 Degree Refinement

The `Angle Precision` value from the right panel is connected to the engine:

- `RotationMode::FixedStep` uses it as the coarse rotation step, with a safety cap to avoid pathological global sweeps.
- `RotationMode::ContinuousRefine` uses it as the minimum local refinement step.

When the user enters `0.001`, the solver does not scan all 360,000 possible global angles. Multi-start still works with coarse candidates, then `UltraRefinement` searches only around prioritized parts in the current best layout:

- Fast: `1°`, `0.1°`
- Balanced: `1°`, `0.1°`, `0.01°`
- MaxQuality: `1°`, `0.1°`, `0.01°`, `0.001°`

Each accepted refinement must improve score and remain valid: no part collision, no spacing violation, no sheet invalidity, and no forbidden-zone violation.

## Known Gaps

- Clearance is conservative segment-distance validation, not true polygon offset.
- DXF/PLT import support is intentionally small and ASCII-oriented.
- Custom sheet containment is stronger for concave boundaries, holes, and forbidden zones, but sheet margin is still conservative and not a true inward offset.
- User placement points are model/engine-ready, but canvas click editing is not enabled yet.
- Ultra refinement is active, but it is intentionally local and prioritized; it is not a full continuous nonlinear optimizer.
- Direct2D, GPU evaluation, AI import, and Corel macro installation are intentionally not implemented yet.
- The solver is now collision-driven v1 with parallel multi-start, but contact candidates are still geometric heuristics rather than a full NFP engine.
