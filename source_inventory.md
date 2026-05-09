# Source Inventory

Generated: 2026-05-09

## Source Folder Counts

| Folder | Files | .cpp | .h | .rc |
|---|---:|---:|---:|---:|
| app | 5 | 3 | 2 | 0 |
| bridge | 4 | 2 | 2 | 0 |
| core | 11 | 0 | 11 | 0 |
| engine | 88 | 42 | 46 | 0 |
| geometry | 26 | 13 | 13 | 0 |
| import | 9 | 4 | 5 | 0 |
| localization | 2 | 1 | 1 | 0 |
| render | 4 | 2 | 2 | 0 |
| resources | 2 | 0 | 1 | 1 |
| tests | 62 | 60 | 2 | 0 |
| ui | 13 | 6 | 7 | 0 |

## Engine Files

- acceptance.cpp
- acceptance.h
- adaptive_acceptance.cpp
- adaptive_acceptance.h
- adaptive_optimizer.cpp
- adaptive_optimizer.h
- aggressive_gap_filler.cpp
- aggressive_gap_filler.h
- analytic_contact_candidate.cpp
- analytic_contact_candidate.h
- broadphase.cpp
- broadphase.h
- candidate_cache.h
- compression.cpp
- compression.h
- constructive_rebuild_engine.cpp
- constructive_rebuild_engine.h
- contact_candidate_provider.cpp
- contact_candidate_provider.h
- contact_packing.cpp
- contact_packing.h
- convergence.cpp
- convergence.h
- destroy_rebuild.cpp
- destroy_rebuild.h
- empty_space_map.cpp
- empty_space_map.h
- engine_settings.h
- escape_search.cpp
- escape_search.h
- free_space_analyzer.cpp
- free_space_analyzer.h
- frontier_analyzer.cpp
- frontier_analyzer.h
- gap_filling.cpp
- gap_filling.h
- gpu_evaluator.h
- guided_local_search.cpp
- guided_local_search.h
- inner_fit_candidate_provider.cpp
- inner_fit_candidate_provider.h
- layout_eval_cache.cpp
- layout_eval_cache.h
- layout_score.cpp
- layout_score.h
- layout_score_components.cpp
- layout_score_components.h
- layout_state.cpp
- layout_state.h
- local_region_repack.cpp
- local_region_repack.h
- local_search.cpp
- local_search.h
- multi_start_solver.cpp
- multi_start_solver.h
- narrowphase.cpp
- narrowphase.h
- nesting_engine.cpp
- nesting_engine.h
- nfp_cache.cpp
- nfp_cache.h
- nfp_candidate_provider.cpp
- nfp_candidate_provider.h
- nfp_solver_cache.cpp
- nfp_solver_cache.h
- overlap_resolver.cpp
- overlap_resolver.h
- parallel_collision_evaluator.cpp
- parallel_collision_evaluator.h
- penalty_system.cpp
- penalty_system.h
- pose_sampler.cpp
- pose_sampler.h
- rearrangement.cpp
- rearrangement.h
- region_repack.cpp
- region_repack.h
- slide_to_contact.cpp
- slide_to_contact.h
- small_part_filler.cpp
- small_part_filler.h
- solver_state.h
- tabu_memory.cpp
- tabu_memory.h
- ultra_refinement.cpp
- ultra_refinement.h
- worker_pool.cpp
- worker_pool.h

## Geometry Files

- arc.cpp
- arc.h
- bezier.cpp
- bezier.h
- clearance.cpp
- clearance.h
- collision.cpp
- collision.h
- flatten.cpp
- flatten.h
- inner_fit_polygon.cpp
- inner_fit_polygon.h
- minkowski_sum.cpp
- minkowski_sum.h
- no_fit_polygon.cpp
- no_fit_polygon.h
- polygon_offset.cpp
- polygon_offset.h
- polygon_utils.cpp
- polygon_utils.h
- spatial_grid.cpp
- spatial_grid.h
- transformed_shape.cpp
- transformed_shape.h
- winding.cpp
- winding.h

## UI Files

- canvas_view.cpp
- canvas_view.h
- control_ids.h
- layout.cpp
- layout.h
- left_panel.cpp
- left_panel.h
- progress_bar.cpp
- progress_bar.h
- right_panel.cpp
- right_panel.h
- toolbar.cpp
- toolbar.h

## Test Files

- acceptance_model_smoke.cpp
- active_contact_depth_smoke.cpp
- adaptive_part_need_smoke.cpp
- analytic_contact_candidate_smoke.cpp
- architecture_audit_smoke.cpp
- beam_search_quality_smoke.cpp
- benchmark_consistency_smoke.cpp
- benchmark_runner.cpp
- clearance_smoke.cpp
- cluster_compaction_smoke.cpp
- cluster_reinsert_quality_smoke.cpp
- collision_tests.cpp
- constructive_rebuild_smoke.cpp
- contour_interlock_smoke.cpp
- contour_locking_nfp_smoke.cpp
- convergence_quality_smoke.cpp
- convergence_smoke.cpp
- coordinated_cluster_motion_smoke.cpp
- delta_score_smoke.cpp
- dense_nfp_packing_smoke.cpp
- dense_small_part_compaction_smoke.cpp
- destroy_best_update_smoke.cpp
- destroy_rebuild_smoke.cpp
- dirty_region_render_smoke.cpp
- empty_region_targeting_smoke.cpp
- escape_smoke.cpp
- fake_stats_guard_smoke.cpp
- fit_candidate_generation_smoke.cpp
- gap_filling_smoke.cpp
- geometry_collision_smoke.cpp
- geometry_tests.cpp
- hole_aware_nfp_smoke.cpp
- industrial_quality_smoke.cpp
- inner_fit_candidate_provider_smoke.cpp
- inner_fit_polygon_smoke.cpp
- large_scale_quality_smoke.cpp
- local_region_repack_smoke.cpp
- minkowski_sum_smoke.cpp
- multi_delta_smoke.cpp
- multi_part_constructive_quality_smoke.cpp
- nfp_cache_real_smoke.cpp
- nfp_cache_smoke.cpp
- nfp_candidate_provider_smoke.cpp
- nfp_constructive_rebuild_smoke.cpp
- nfp_provider_test_common.h
- nfp_rebuild_quality_smoke.cpp
- no_bbox_shelf_smoke.cpp
- no_fit_polygon_smoke.cpp
- no_global_mode_smoke.cpp
- parser_tests.cpp
- partial_cluster_state_smoke.cpp
- performance_stress_smoke.cpp
- placement_start_smoke.cpp
- polygon_offset_smoke.cpp
- rearrangement_smoke.cpp
- rebuild_objective_smoke.cpp
- score_calibration_smoke.cpp
- solver_500_max_spacing0_smoke.cpp
- solver_quality_smoke.cpp
- solver_smoke_common.h
- temporary_acceptance_guard_smoke.cpp
- ultra_refinement_smoke.cpp

## KEEP

- core, geometry/collision, geometry/clearance, geometry/transformed_shape: correctness-critical exact validation layer.
- geometry/minkowski_sum, geometry/no_fit_polygon, geometry/inner_fit_polygon, geometry/polygon_offset: current NFP/IFP geometry foundation.
- engine/multi_start_solver, engine/constructive_rebuild_engine, engine/adaptive_optimizer: current active solve path.
- engine/layout_score, engine/layout_eval_cache, engine/broadphase, engine/parallel_collision_evaluator, engine/worker_pool: scoring, cache and validation performance path.
- engine/contact_candidate_provider, engine/nfp_candidate_provider, engine/inner_fit_candidate_provider, engine/analytic_contact_candidate: active provider chain.
- pp, ui, ender, localization, ridge: application, Win32 UI/render/localization/Corel bridge boundaries.
- Geometry/provider smoke tests and benchmark runner: useful when they execute real code.

## DEPRECATE CANDIDATE

- engine/compression, engine/gap_filling, engine/rearrangement, engine/region_repack, engine/local_region_repack, engine/aggressive_gap_filler, engine/small_part_filler: older phase/operator modules. They still compile and may be referenced by tests or helper paths, but the intended main quality path is now constructive rebuild plus micro adaptive refine.
- engine/contact_packing, engine/frontier_analyzer, engine/free_space_analyzer, engine/escape_search, engine/tabu_memory: useful concepts but should be merged behind the constructive-rebuild/provider model or kept as bounded helper modules.
- engine/nfp_cache: old candidate-list cache. engine/nfp_solver_cache is the newer real-loop cache; keep both until provider migration is finished, then merge/remove the old cache.
- Source-string smoke tests such as architecture/depth/guard checks: keep as audit guards but do not count as quality validation.

## REMOVE LATER

- Any tracked outputs/benchmark/*.cignest.json and *.result.svg exports should remain removed and regenerated outside git.
- Archive artifacts such as root zip packages should stay ignored and out of source control.
- CMake manual duplicate source lists should be refactored into shared source groups instead of repeated per target.
