# Project Health Audit

Generated: 2026-05-09

This audit intentionally does not change solver algorithms, UI behavior, NFP/IFP algorithms, or benchmark quality logic. It records the current project state, cleanup performed, architectural risks, and recommended next engineering work.

## Current Architecture

CigerNesting is still structurally separated into Win32 app/UI/render layers, portable core/geometry/import/engine layers, bridge scaffolding, localization, and tests. The executable target is a native C++20 Win32 application with GDI rendering and no external geometry/UI dependency.

Current motor reality:

- Exact solid collision, hole-aware containment, clearance validation, sheet containment, and broadphase/narrowphase separation are present.
- Solver correctness is protected by full LayoutScore/final validation; invalid final layouts are not supposed to be accepted.
- The active search has moved away from pure row/shelf placement. However, quality is still not industrial: accepted NFP/IFP/contact candidates do not reliably produce global used-area reduction.
- Recent real NFP/IFP primitives are present, but the NFP core is not yet production-complete for concave industrial nesting because concave Minkowski components are not union-cleaned into final arrangement regions.
- The repository had become dominated by generated artifacts, not source: before cleanup, build outputs represented roughly 95% of repo size.

## Active Solver Path

Observed active high-level solver path:

1. `NestingEngine` runs solver work outside the UI thread.
2. `MultiStartSolver` creates a valid baseline using `contourSeedBaseline`.
3. For non-Fast profiles, `ConstructiveRebuildEngine::optimize` is called as the main quality stage.
4. `ConstructiveRebuildEngine` uses the provider chain:
   - `InnerFitCandidateProvider`
   - `NfpCandidateProvider`
   - `AnalyticContactCandidateProvider` fallback
   - restore fallback inside beam completion
5. `AdaptiveUnifiedOptimizer` runs after constructive rebuild as micro/fine adjustment.
6. Final stats and validation are refreshed before returning the result.

Important path issue:

- In `multi_start_solver.cpp`, constructive rebuild progress is still surfaced through legacy labels such as `SolverPhase::Rearrangement` and `SolverStrategy::RegionRepack`, while adaptive progress is surfaced through `SolverPhase::Exploration`. This is a UI/progress vocabulary mismatch and can make the new architecture look like old phases.

## Geometry/NFP/IFP Status

Current geometry status:

- `geometry/collision.*`: active exact solid collision layer.
- `geometry/clearance.*`: active exact/conservative segment-distance clearance layer.
- `geometry/transformed_shape.*`: active transformed ring/part cache input for collision/clearance.
- `geometry/minkowski_sum.*`: real Minkowski support exists.
- `geometry/no_fit_polygon.*`: real NFP component loop generation exists.
- `geometry/inner_fit_polygon.*`: rectangular-sheet IFP exists.
- `geometry/polygon_offset.*`: conservative offset v1 exists for candidate generation.
- `engine/nfp_solver_cache.*`: real NFP loop cache exists.

NFP/IFP maturity:

- Convex polygon NFP path is a real Minkowski boundary/component path.
- Concave polygon support currently uses ear-clipping decomposition and returns component loops.
- Missing: polygon union/self-intersection cleanup for decomposed concave Minkowski components.
- Missing: robust hole-aware NFP region composition. Hole candidates exist, but they are still conservative and validation-backed.
- Rectangular IFP is usable and exact for translated part-origin feasibility.
- Custom sheet IFP remains conservative and relies on exact validation fallback.
- Offset is candidate-generation support only; final correctness still depends on exact clearance and containment validation.

Recent quality smoke data before this audit showed:

- `mixed_100`: valid but QUALITY_FAIL, utilization around `0.532`, `clusterBeamAccepted=0`.
- `many_small`: valid but QUALITY_FAIL, utilization around `0.598`, `denseClusterBeamAccepted=0`.
- `mixed_500`: long-running / non-convergent risk under heavier smoke paths.

## Known Quality Bottlenecks

Primary technical blockers:

1. Concave NFP decomposition is not union-cleaned.
2. NFP candidates are generated and validated, but cluster beam leaves are not accepted because they do not produce enough global used-area/utilization improvement.
3. `clusterBeamAccepted=0` and `denseClusterBeamAccepted=0` indicate candidate quality/region construction is still insufficient, not simply candidate count.
4. Restore fallback and partial leaf completion can finish beams without meaningful global compaction.
5. Empty-space targeting still does not turn valid candidate sets into robust local reinsert/compaction wins.
6. `AdaptiveUnifiedOptimizer` remains useful for micro moves, but cannot compensate for missing NFP/IFP region quality.
7. Many general large-part benchmarks remain valid but visually sparse or column/shelf-like.
8. The 500-part smoke can run too long after real NFP generation enters the path, so future NFP work needs bounded region construction and cache-aware sampling.

What is not the main blocker anymore:

- Basic candidate generation exists.
- Exact validation exists.
- Threaded/background solve architecture exists.
- The current failure is mostly geometry-region quality plus constructive-rebuild objective/leaf acceptance.

## UI/UX Issues

Known UI/UX issues from source audit:

- Canvas has version and changed-part checks, plus dirty-region invalidation attempts, but several paths still use full `InvalidateRect`.
- Active move summary can display zero-count fallback text, which is technically truthful but not very useful to a user watching a stalled solve.
- Solver progress labels still expose legacy phases/strategies in places, even when constructive rebuild is the actual active engine.
- Full modern visual redesign has not been done; current UI remains functional Win32 controls plus custom canvas.
- Timeout/settings UX is partially improved, but should be rechecked after future UI work to ensure safety cap is not treated as required time limit.
- Debug overlay exists in spirit through changed parts/subset metadata, but visual explanation of rebuild attempts, subset bounds, and NFP candidate sources is still limited.

## Test Reliability

Real execution tests:

- Geometry/collision/clearance smoke tests execute real geometry code.
- New NFP/IFP tests execute real Minkowski/NFP/IFP/cache/provider code.
- Benchmark runner executes real imports, solver, JSON/SVG export, and quality gates.
- Industrial/large-scale quality tests execute real solver paths and can correctly report QUALITY_FAIL.

Guard/string/audit-style tests:

- `architecture_audit_smoke.cpp`
- `active_contact_depth_smoke.cpp`
- `beam_search_quality_smoke.cpp`
- `placement_start_smoke.cpp`
- `rebuild_objective_smoke.cpp`
- `temporary_acceptance_guard_smoke.cpp`
- Parts of `fake_stats_guard_smoke.cpp`

These are useful as regression guards against accidental architecture rollback, but they are not quality proof. They should be labeled as audit guards, not solver quality tests.

Reliability concerns:

- There are many smoke targets, but not all exercise the full real solver.
- Some tests check source-code structure instead of behavior. Keep them, but do not use them as benchmark acceptance.
- `benchmark_results.csv` canonical state is currently absent after cleanup; benchmark runner can regenerate it.
- `outputs/benchmark/*` was tracked/generated output and has now been removed from the working tree. It should not be committed again.

## CMake/Build Health

Current CMake state:

- `NestingApp/CMakeLists.txt` has about 58 executable targets.
- Manual source entries counted in CMake: about 1180.
- Duplicate unique source entries: about 57.
- High-repeat examples include `geometry/clearance.cpp`, `geometry/collision.cpp`, `geometry/transformed_shape.cpp`, `engine/layout_score.cpp`, `engine/broadphase.cpp`, and newer NFP sources.

Risk:

- The repeated per-target source lists are fragile. Adding one new engine/geometry file requires editing many locations.
- New NFP files were added to many repeated lists; this works but increases maintenance risk.
- Large CMake refactor was intentionally not done in this cleanup pass.

Recommended CMake direction:

- Create shared source variables/libraries for core geometry, engine common, import common, and benchmark/smoke common.
- Keep small tests linked against minimal source sets where possible.
- Separate audit/string-guard tests from real solver tests.
- Add a CMake function for smoke targets to avoid repeated include/compile definitions.

## Repository Size Problems

Before cleanup:

- Total repo working tree size: about `1.12 GB`.
- `build/`: about `1.06 GB`.
- `outputs/`: about `1.0 MB`.
- Artifact count found: about `2175` build/export/archive files.
- Largest files were mostly `.pdb`, `.obj`, `.exe`, `.ilk`, and generated Visual Studio/CMake outputs.
- A root zip artifact existed and was removed.

After cleanup:

- Total repo working tree size: about `57.75 MB` after report generation and CMake configure verification.
- `build/`: removed. A quick Visual Studio 2022 x64 CMake configure was run successfully after cleanup, then `build/` was removed again.
- `outputs/`: removed; generated benchmark exports are now absent from the working tree and ignored for future runs.
- Remaining largest directory is `.git` at about `56.47 MB`.
- Remaining source tree is small: `NestingApp` about `1.16 MB`, `samples` about `80 KB`.

Removed safely:

- `build/`
- `outputs/`
- root zip artifacts such as tracked `28.zip` and untracked/archive zip files
- generated benchmark JSON/SVG exports under `outputs/benchmark`
- generated build logs under `outputs`

`.gitignore` was strengthened to ignore:

- build/output folders
- Visual Studio folders
- executable/debug/object/library artifacts
- archives
- generated nesting/benchmark exports
- logs/temp/editor noise

## Deprecated or Duplicate Modules

Keep / active:

- `multi_start_solver`
- `constructive_rebuild_engine`
- `adaptive_optimizer` as micro refinement
- `layout_score`, `layout_eval_cache`, `broadphase`, `parallel_collision_evaluator`, `worker_pool`
- `nfp_candidate_provider`, `inner_fit_candidate_provider`, `contact_candidate_provider`, `analytic_contact_candidate`
- `collision`, `clearance`, `transformed_shape`, `minkowski_sum`, `no_fit_polygon`, `inner_fit_polygon`, `polygon_offset`

Deprecate candidate / merge later:

- `compression`
- `gap_filling`
- `rearrangement`
- `region_repack`
- `local_region_repack`
- `aggressive_gap_filler`
- `small_part_filler`
- `frontier_analyzer`
- `free_space_analyzer`
- `escape_search`
- `tabu_memory`
- `contact_packing`

Reason:

- These modules reflect earlier operator/phase experiments. Some still compile and may feed helper/test paths, but the intended industrial path is constructive rebuild plus provider-chain geometry.

Duplicate/cache cleanup candidate:

- `engine/nfp_cache.*` stores old candidate-list style data.
- `engine/nfp_solver_cache.*` stores real NFP loops.
- Keep both for now, but plan a merge once all provider paths use the real loop cache.

## Recommended Next Engineering Steps

Next NFP/IFP iteration should focus on geometry correctness and region quality, not another heuristic layer:

1. Implement polygon union/arrangement cleanup for concave Minkowski component loops.
2. Convert decomposed NFP components into usable non-overlapping no-fit regions with loop classification.
3. Add robust self-intersection cleanup and collinear edge normalization after Minkowski operations.
4. Extend IFP from rectangular exact region to custom sheet outer contour with holes/forbidden-zone region subtraction.
5. Add NFP/IFP region intersection with EmptySpaceMap targets, so candidate selection is driven by actual fillable region coverage.
6. Make cluster beam leaf score use real NFP/IFP region coverage and used-bounds shrink, not just contact/valid candidate count.
7. Bound 500-part NFP path with cache-aware owner selection, region-local NFP generation, and strict candidate budgets.
8. Refactor CMake into common source groups before adding more test targets.
9. Rename/source-label guard tests so benchmark quality dashboards do not confuse source-string guards with real solver validation.
10. Keep `outputs/` and generated exports out of git; regenerate benchmark reports only when explicitly requested.

