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
- `EngineSettings::performanceProfile` controls runtime depth separately from geometric validity
- `PenaltySystem` separates per-attempt pair weights from optional low-weight global bias
- `AcceptanceCriteria` adds profile-aware simulated annealing so valid but temporarily worse single-part moves can help escape local minima
- `TabuMemory` keeps approximate recent move/layout hashes to avoid short repeated loops
- `PoseSampler` generates translation, sheet contact, neighbor-edge contact, hole/forbidden-zone contact, rotation, mirror, and random jump candidates
- `GuidedLocalSearch` tries candidate moves for colliding parts, uses delta scoring first, applies annealing acceptance, and still full-validates accepted candidates
- `OverlapResolver` runs guided collision resolution while allowing temporarily invalid/overlapping layouts during exploration
- `Compression` searches for safe movement with large-to-small step refinement, profile-aware shelf repack, horizontal/vertical sweep compaction, alternating XY passes, and strategy-aware movement directions
- `FrontierAnalyzer` extracts skyline/frontier candidates from sheet bounds, current used edges, common part bbox coordinates, sparse grid cells, and contact snap lines
- `SmallPartFiller` targets small or fringe parts and tries to move them into frontier/gap candidates without relaxing collision, clearance, or sheet validation
- `RegionRepack` identifies sparse low-density regions in the used bounds and attempts a bounded local repack around those regions
- `FreeSpaceAnalyzer` extracts sheet, contour, hole, concavity, used-bounds, and forbidden-zone placement opportunities from the current layout
- `GapFilling` moves small parts into high-value cavities after compression, using delta scoring first and full contour validation before accepting a move
- `Rearrangement` tries valid multi-part moves after gap filling: swaps, short ejection chains, and small cluster compaction
- `EscapeSearch` deliberately disperses a few parts when the search stalls, then the normal optimizers pull the layout back toward a better valid state
- `MultiStartSolver` runs attempts in parallel through the internal worker pool, cycling placement strategies, seeds, and part orderings within `timeLimitSeconds`, while preserving best-so-far
- `UltraRefinement` refines the best layout locally with angle ladders, micro-translation correction, optional mirroring, and strict contour validation
- `SolverStats` reports evaluated candidates, accepted moves, worse accepted/rejected moves, tabu rejections, escape use, worker count, cache hits, compaction/frontier/filler/repack counts, swap/chain/cluster move counts, elapsed time, and candidates/second
- `LayoutEvalCache` supports incremental delta scoring for single-part candidate moves, avoiding full layout rescoring for every rejected candidate
- `LayoutEvalCache` also supports bounded multi-part delta scoring for swap, chain, cluster, and region-repack style moves

Solver quality smoke test target:

```powershell
build\NestingApp\Release\CigerNestingSolverQualitySmoke.exe <repo-root>
```

Performance stress smoke target:

```powershell
build\NestingApp\Release\CigerNestingPerformanceStressSmoke.exe
```

Large-scale quality smoke target:

```powershell
build\NestingApp\Release\CigerNestingLargeScaleQualitySmoke.exe <repo-root>
```

Delta scoring smoke target:

```powershell
build\NestingApp\Release\CigerNestingDeltaScoreSmoke.exe
```

Multi-part delta scoring smoke target:

```powershell
build\NestingApp\Release\CigerNestingMultiDeltaSmoke.exe
```

Gap filling smoke target:

```powershell
build\NestingApp\Release\CigerNestingGapFillingSmoke.exe
```

Rearrangement smoke target:

```powershell
build\NestingApp\Release\CigerNestingRearrangementSmoke.exe
```

Escape/adaptive search smoke target:

```powershell
build\NestingApp\Release\CigerNestingEscapeSmoke.exe
```

The stress smoke generates 100, 250, and 500-part synthetic documents and reports Fast/Balanced/Maximum profile throughput. It is intentionally bounded so it can run as a smoke test; full long-running benchmark sweeps should use the main solver with larger `timeLimitSeconds`.

## Delta Scoring

`LayoutEvalCache` stores the current layout's transformed parts, part bounds, broadphase pair contributions, sheet validity, used bounds, and a lightweight grid index. When one candidate moves one part, `evaluateMoveDelta` recalculates only:

- the moved part
- previous pair contributions involving the moved part
- new expanded-AABB neighbors from the spatial grid
- the moved part's sheet validity and margin
- used bounds, with full used-bounds recompute only when the moved part touched the previous used-layout boundary

Guided local search, score compression, ultra refinement, and performance stress candidate checks use delta scoring first. Accepted candidates are still verified with full `LayoutScore` before becoming the new state, so validity rules are not relaxed.

For coupled moves, `evaluateMultiMoveDelta` accepts a `MultiDeltaMove` containing multiple part indices and new poses. It builds an affected set from:

- moved parts
- old cached neighbors
- new spatial-grid neighbors
- moved-to-moved pairs

Only pair contributions involving moved parts are removed and recalculated. Unaffected pair, ring, transformed-part, and sheet contributions stay cached. Accepted multi-part moves update only moved transformed parts, sheet validity, spatial-grid cells, and affected pair entries. Used bounds are updated incrementally unless one moved part touched the previous used-boundary, in which case the used bounds are recomputed safely.

Pair cache entries store collision contribution, spacing contribution, clearance validity, and last boundary distance. Exact clearance is skipped for obviously far AABB-separated pairs; near pairs still run exact contour clearance. Every accepted delta candidate is still verified by full `LayoutScore`, and delta/full score mismatches fall back to the full score result.

## Gap Filling

The solver now has a cavity-aware pass between compression and ultra refinement:

- sheet corners and boundaries seed conservative anchors
- part outer contour bounds generate contact-oriented anchors
- part hole rings generate center, inset-corner, and edge-midpoint anchors
- concave vertices generate local indentation anchors without requiring a full convex hull/NFP implementation
- used layout bounds contribute internal gap anchors
- forbidden zones contribute avoidance/contact anchors around their safe perimeter

The pass targets smaller parts first, roughly the lower 30% by footprint, so holes in donut/B-like parts and concave notches are tried before spending budget on large pieces. Each candidate is evaluated with `LayoutEvalCache::evaluateMoveDelta`; accepted candidates are then rechecked with full `LayoutScore`, real contour collision, clearance, sheet containment, and sheet margin rules.

`LayoutScore` also includes a small cavity-placement reward when a part is validly inside another part's hole. This does not relax collision or clearance. It only helps the search prefer equally valid compact layouts that actively use internal voids.

## General Large-Scale Quality

The solver has an additional set of general-purpose quality passes for mixed jobs that do not contain obvious holes or concavities:

- strengthened strip/line compaction runs shelf repack first, then safe maximum shifts in horizontal and vertical sweeps
- alternating XY compaction changes axis order between passes so row-biased layouts can still shrink vertically
- frontier analysis generates candidates from sheet sides, current layout frontiers, bbox edge alignments, skyline cells, and contact snap positions
- small-part filler can temporarily move selected small/fringe parts into those frontier candidates and accepts the move only after full validation
- region repack splits the used area into a grid, finds sparse cells, and tries a bounded local repack for parts around those cells

These passes are intentionally heuristic, but they are not shortcuts around validation. Accepted results must still have zero collision, zero spacing penalty, zero sheet penalty, and zero invalid parts.

## Rearrangement

The solver now includes a multi-part rearrangement pass between gap filling and ultra refinement:

- swap moves exchange two small/medium parts by exact pose and center-preserving variants, then require full `LayoutScore` validity
- ejection-chain v1 moves a small part into a hole/gap/concavity anchor, identifies a small set of conflicting parts, and relocates those conflicts to nearby free-space anchors
- cluster compaction picks 3-8 nearby parts and tries safe group movement plus intra-cluster shrinking to reduce used area

Profile budgets:

- `Fast`: shallow pass, chain depth 1, small swap/cluster budgets
- `Balanced`: chain depth 2 and moderate swap/cluster budgets
- `Maximum`: chain depth 3, up to 5 affected parts, larger anchor and cluster budgets

Multi-part moves use full-score fallback instead of pretending they are single-part delta moves. A candidate is accepted only if the final layout is valid: zero collision, zero spacing penalty, zero invalid parts, and valid sheet containment.

## Adaptive Search

The local optimizer now has a controlled local-minimum escape layer:

- simulated annealing acceptance can temporarily accept a worse single-part move when the candidate is delta-valid
- temperature cools over the local-search iteration budget and is profile-sensitive
- `Fast` has very low annealing, `Balanced` uses moderate annealing, and `Maximum` is intentionally more exploratory
- tabu memory blocks recently seen approximate moves and layouts, reducing short oscillation loops
- adaptive pair penalties raise the cost of repeatedly colliding pairs, mark stalled pairs, and gently decay resolved pairs
- the `Escape` phase scatters a few validly movable parts, then gap filling/rearrangement/local optimization attempts to recover a better layout

The best-so-far result is still protected. Worse accepted moves can affect the current search trajectory, but the solver only promotes a layout to best when full scoring says it is valid and better.

## Performance Profiles

`PerformanceProfile` is independent from validity. Every profile still requires zero collision, zero spacing violation, and valid sheet containment.

- `Fast`: fewer attempts, shallower local search, smaller neighbor contact set, half of automatic logical CPU count.
- `Balanced`: more attempts, more candidate evaluation, automatic CPU count leaves one logical core free.
- `Maximum`: full automatic logical CPU count, deepest attempt/refinement budgets, largest contact candidate set.

When `cpuThreadCount` is `0`, the solver resolves thread count from the profile. Setting `cpuThreadCount` manually still overrides the profile thread count.

Ultra refinement smoke test target:

```powershell
build\NestingApp\Release\CigerNestingUltraRefinementSmoke.exe
```

## Benchmark Suite

The repository includes a deterministic benchmark corpus under:

```text
samples\benchmark
```

Current benchmark cases:

- `many_small_parts.svg`
- `mixed_100_parts.svg`
- `mixed_250_parts.svg`
- `mixed_500_parts.svg`
- `donut_hole_usage.svg`
- `b_shape_hole_usage.svg`
- `concave_cavity_usage.svg`
- `irregular_sheet_with_hole.svg`
- `mirror_required.svg`
- `rotation_precision_required.svg`

Run the benchmark runner from the repository root:

```powershell
build\NestingApp\Release\CigerNestingBenchmarkRunner.exe .
```

The runner executes each case with `Fast`, `Balanced`, and `Maximum` profiles and writes:

```text
outputs\benchmark\benchmark_results.csv
outputs\benchmark\<case>_<profile>.cignest.json
outputs\benchmark\<case>_<profile>.result.svg
```

The CSV contains the quality/performance columns used for regression tracking:

```text
caseName,partCount,profile,elapsedMs,utilization,usedWidth,usedHeight,collisions,invalid,spacingPenalty,sheetPenalty,bestUpdates,evaluatedCandidates,candidatesPerSecond,acceptedMoves,acceptedWorseMoves,gapAccepted,swapAccepted,chainAccepted,escapeAccepted,ultraAccepted,compactionAccepted,frontierCandidates,smallFillerAccepted,regionRepackAccepted,finalScore,status
```

Each benchmark run is strict about validity:

- `collisions == 0`
- `invalid == 0`
- `spacingPenalty == 0`
- `sheetPenalty == 0`
- `Maximum` must not produce a worse final score than `Fast`
- large-scale quality goals require `many_small_parts`, `mixed_100_parts`, and `mixed_500_parts` to produce best-so-far improvements in `Maximum`

Benchmark settings run with `EngineSettings::deterministic = true`, `randomSeed` fixed per case, and `cpuThreadCount = 1`. This intentionally favors reproducibility over maximum throughput. The runner also repeats one deterministic case internally and fails if the same input/seed/profile produces a different scored layout.

The `.cignest.json` export stores input path, settings, sheet metadata, part count, final poses, score, and solver stats. This is the current app-native result handoff format and is intended to become the bridge foundation for future Corel/export workflows. The `.result.svg` export is a lightweight visual debug view generated by the in-repo SVG writer, without external libraries.

## 0.001 Degree Refinement

The `Angle Precision` value from the right panel is connected to the engine:

- `RotationMode::FixedStep` uses it as the coarse rotation step, with a safety cap to avoid pathological global sweeps.
- `RotationMode::ContinuousRefine` uses it as the minimum local refinement step.

When the user enters `0.001`, the solver does not scan all 360,000 possible global angles. Multi-start still works with coarse candidates, then `UltraRefinement` searches only around prioritized parts in the current best layout:

- Fast: `1°`, `0.1°`
- Balanced: `1°`, `0.1°`, `0.01°`
- MaxQuality: `1°`, `0.1°`, `0.01°`, `0.001°`

Each accepted refinement must improve score and remain valid: no part collision, no spacing violation, no sheet invalidity, and no forbidden-zone violation.

## Constructive Rebuild Engine

`Maximum` quality now treats constructive rebuild as the main quality search, not as a shallow post-pass. The engine targets empty regions, low-contact clusters, and boundary contributors, then rebuilds multi-part subsets with analytic contour-contact candidates. The current analytic provider goes through `IContactCandidateProvider`, so a future NFP/IFP provider can replace or augment it without wiring the main solver directly to one candidate implementation.

Current defaults favor industrial solving:

- `QualityMode::MaxQuality`
- `PerformanceProfile::Maximum`
- `RotationMode::ContinuousRefine`
- `rotationStepDegrees = 0.001`
- `timeLimitSeconds = 0`, meaning auto convergence with positive values used only as a safety cap
- `cpuThreadCount = 0`, meaning profile-driven maximum/automatic CPU selection

NFP/IFP preparation checklist:

1. robust polygon offset prerequisite
2. Minkowski sum support
3. no-fit polygon generation
4. inner-fit polygon generation for sheet boundaries
5. holes and forbidden-zone handling
6. rotation/mirror candidate cache
7. pairwise NFP cache behind `IContactCandidateProvider`

## Known Gaps

- Clearance is conservative segment-distance validation, not true polygon offset.
- DXF/PLT import support is intentionally small and ASCII-oriented.
- Custom sheet containment is stronger for concave boundaries, holes, and forbidden zones, but sheet margin is still conservative and not a true inward offset.
- User placement points are model/engine-ready, but canvas click editing is not enabled yet.
- Ultra refinement is active, but it is intentionally local and prioritized; it is not a full continuous nonlinear optimizer.
- Direct2D, GPU evaluation, AI import, and Corel macro installation are intentionally not implemented yet.
- The solver is now collision-driven v1 with parallel multi-start, but contact candidates are still geometric heuristics rather than a full NFP engine.
- Gap filling is hole-aware and concavity-aware, but concavity detection is currently based on local reflex vertices rather than a full medial-axis/free-space decomposition.
- Rearrangement uses bounded swap/ejection/cluster heuristics with multi-part delta pre-evaluation. It is not yet a full combinatorial optimizer.
- Adaptive search currently applies annealing to single-part delta moves; multi-part adaptive acceptance still uses full-score safety checks and bounded escape heuristics.
- Performance stress smoke is bounded and deterministic; it is not a substitute for long industrial benchmark runs.
- Benchmark runner uses deterministic single-thread settings, so its `candidatesPerSecond` is for regression comparison rather than peak CPU throughput.
- Multi-part delta scoring is active for bounded coupled moves, but accepted moves still require full verification. Future work can extend the same cache model to deeper chain planning and GPU candidate evaluation.
