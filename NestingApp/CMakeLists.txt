set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB_RECURSE CIGER_NESTING_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/app/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ui/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/render/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/geometry/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/import/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/engine/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/bridge/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/localization/*.cpp"
)

file(GLOB_RECURSE CIGER_NESTING_RESOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/resources/*.rc"
)

add_executable(CigerNesting WIN32 ${CIGER_NESTING_SOURCES} ${CIGER_NESTING_RESOURCES})

target_include_directories(CigerNesting PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNesting PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)

target_link_libraries(CigerNesting PRIVATE user32 gdi32 comdlg32 shell32)

add_executable(CigerNestingGeometryCollisionSmoke
    tests/geometry_collision_smoke.cpp
    geometry/clearance.cpp
    geometry/collision.cpp
    geometry/transformed_shape.cpp
)

add_executable(CigerNestingSolverQualitySmoke
    tests/solver_quality_smoke.cpp
    import/importer.cpp
    import/svg_parser.cpp
    import/plt_parser.cpp
    import/dxf_parser.cpp
    geometry/arc.cpp
    geometry/bezier.cpp
    geometry/clearance.cpp
    geometry/flatten.cpp
    geometry/winding.cpp
    geometry/polygon_utils.cpp
    geometry/collision.cpp
    geometry/transformed_shape.cpp
    geometry/spatial_grid.cpp
    engine/acceptance.cpp
    engine/analytic_contact_candidate.cpp
    engine/contact_candidate_provider.cpp
    engine/nfp_cache.cpp
    engine/nfp_candidate_provider.cpp
    engine/inner_fit_candidate_provider.cpp
    engine/adaptive_acceptance.cpp
    engine/destroy_rebuild.cpp
    engine/constructive_rebuild_engine.cpp
    engine/adaptive_optimizer.cpp
    engine/broadphase.cpp
    engine/compression.cpp
    engine/contact_packing.cpp
    engine/convergence.cpp
    engine/escape_search.cpp
    engine/frontier_analyzer.cpp
    engine/free_space_analyzer.cpp
    engine/gap_filling.cpp
    engine/guided_local_search.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/layout_state.cpp
    engine/multi_start_solver.cpp
    engine/nesting_engine.cpp
    engine/overlap_resolver.cpp
    engine/parallel_collision_evaluator.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
    engine/rearrangement.cpp
    engine/region_repack.cpp
    engine/local_region_repack.cpp
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
    engine/aggressive_gap_filler.cpp
    engine/empty_space_map.cpp
    engine/slide_to_contact.cpp
    engine/worker_pool.cpp
)

add_executable(CigerNestingUltraRefinementSmoke
    tests/ultra_refinement_smoke.cpp
    geometry/clearance.cpp
    geometry/collision.cpp
    geometry/transformed_shape.cpp
    geometry/spatial_grid.cpp
    engine/broadphase.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
    engine/layout_eval_cache.cpp
    engine/ultra_refinement.cpp
    engine/aggressive_gap_filler.cpp
    engine/empty_space_map.cpp
    engine/slide_to_contact.cpp
    engine/worker_pool.cpp
)

add_executable(CigerNestingClearanceSmoke
    tests/clearance_smoke.cpp
    geometry/clearance.cpp
    geometry/collision.cpp
    geometry/transformed_shape.cpp
)

add_executable(CigerNestingPerformanceStressSmoke
    tests/performance_stress_smoke.cpp
    geometry/clearance.cpp
    geometry/collision.cpp
    geometry/spatial_grid.cpp
    geometry/transformed_shape.cpp
    engine/acceptance.cpp
    engine/analytic_contact_candidate.cpp
    engine/contact_candidate_provider.cpp
    engine/nfp_cache.cpp
    engine/nfp_candidate_provider.cpp
    engine/inner_fit_candidate_provider.cpp
    engine/adaptive_acceptance.cpp
    engine/destroy_rebuild.cpp
    engine/constructive_rebuild_engine.cpp
    engine/adaptive_optimizer.cpp
    engine/broadphase.cpp
    engine/compression.cpp
    engine/contact_packing.cpp
    engine/convergence.cpp
    engine/escape_search.cpp
    engine/frontier_analyzer.cpp
    engine/free_space_analyzer.cpp
    engine/gap_filling.cpp
    engine/guided_local_search.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/layout_state.cpp
    engine/multi_start_solver.cpp
    engine/nesting_engine.cpp
    engine/overlap_resolver.cpp
    engine/parallel_collision_evaluator.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
    engine/rearrangement.cpp
    engine/region_repack.cpp
    engine/local_region_repack.cpp
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
    engine/aggressive_gap_filler.cpp
    engine/empty_space_map.cpp
    engine/slide_to_contact.cpp
    engine/worker_pool.cpp
)

add_executable(CigerNestingDeltaScoreSmoke
    tests/delta_score_smoke.cpp
    geometry/clearance.cpp
    geometry/collision.cpp
    geometry/spatial_grid.cpp
    geometry/transformed_shape.cpp
    engine/broadphase.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
)

add_executable(CigerNestingMultiDeltaSmoke
    tests/multi_delta_smoke.cpp
    geometry/clearance.cpp
    geometry/collision.cpp
    geometry/spatial_grid.cpp
    geometry/transformed_shape.cpp
    engine/broadphase.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/penalty_system.cpp
)

add_executable(CigerNestingGapFillingSmoke
    tests/gap_filling_smoke.cpp
    geometry/clearance.cpp
    geometry/collision.cpp
    geometry/spatial_grid.cpp
    geometry/transformed_shape.cpp
    engine/broadphase.cpp
    engine/free_space_analyzer.cpp
    engine/gap_filling.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
)

add_executable(CigerNestingRearrangementSmoke
    tests/rearrangement_smoke.cpp
    geometry/clearance.cpp
    geometry/collision.cpp
    geometry/spatial_grid.cpp
    geometry/transformed_shape.cpp
    engine/broadphase.cpp
    engine/free_space_analyzer.cpp
    engine/gap_filling.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
    engine/rearrangement.cpp
)

add_executable(CigerNestingEscapeSmoke
    tests/escape_smoke.cpp
    geometry/clearance.cpp
    geometry/collision.cpp
    geometry/spatial_grid.cpp
    geometry/transformed_shape.cpp
    engine/acceptance.cpp
    engine/analytic_contact_candidate.cpp
    engine/contact_candidate_provider.cpp
    engine/nfp_cache.cpp
    engine/nfp_candidate_provider.cpp
    engine/inner_fit_candidate_provider.cpp
    engine/broadphase.cpp
    engine/escape_search.cpp
    engine/frontier_analyzer.cpp
    engine/free_space_analyzer.cpp
    engine/gap_filling.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
    engine/rearrangement.cpp
    engine/region_repack.cpp
    engine/local_region_repack.cpp
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
)

add_executable(CigerNestingBenchmarkRunner
    tests/benchmark_runner.cpp
    import/importer.cpp
    import/svg_parser.cpp
    import/plt_parser.cpp
    import/dxf_parser.cpp
    geometry/arc.cpp
    geometry/bezier.cpp
    geometry/clearance.cpp
    geometry/flatten.cpp
    geometry/winding.cpp
    geometry/polygon_utils.cpp
    geometry/collision.cpp
    geometry/transformed_shape.cpp
    geometry/spatial_grid.cpp
    engine/acceptance.cpp
    engine/analytic_contact_candidate.cpp
    engine/contact_candidate_provider.cpp
    engine/nfp_cache.cpp
    engine/nfp_candidate_provider.cpp
    engine/inner_fit_candidate_provider.cpp
    engine/adaptive_acceptance.cpp
    engine/destroy_rebuild.cpp
    engine/constructive_rebuild_engine.cpp
    engine/adaptive_optimizer.cpp
    engine/broadphase.cpp
    engine/compression.cpp
    engine/contact_packing.cpp
    engine/convergence.cpp
    engine/escape_search.cpp
    engine/frontier_analyzer.cpp
    engine/free_space_analyzer.cpp
    engine/gap_filling.cpp
    engine/guided_local_search.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/layout_state.cpp
    engine/multi_start_solver.cpp
    engine/overlap_resolver.cpp
    engine/parallel_collision_evaluator.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
    engine/rearrangement.cpp
    engine/region_repack.cpp
    engine/local_region_repack.cpp
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
    engine/aggressive_gap_filler.cpp
    engine/empty_space_map.cpp
    engine/slide_to_contact.cpp
    engine/worker_pool.cpp
)

add_executable(CigerNestingLargeScaleQualitySmoke
    tests/large_scale_quality_smoke.cpp
    import/importer.cpp
    import/svg_parser.cpp
    import/plt_parser.cpp
    import/dxf_parser.cpp
    geometry/arc.cpp
    geometry/bezier.cpp
    geometry/clearance.cpp
    geometry/flatten.cpp
    geometry/winding.cpp
    geometry/polygon_utils.cpp
    geometry/collision.cpp
    geometry/transformed_shape.cpp
    geometry/spatial_grid.cpp
    engine/acceptance.cpp
    engine/analytic_contact_candidate.cpp
    engine/contact_candidate_provider.cpp
    engine/nfp_cache.cpp
    engine/nfp_candidate_provider.cpp
    engine/inner_fit_candidate_provider.cpp
    engine/adaptive_acceptance.cpp
    engine/destroy_rebuild.cpp
    engine/constructive_rebuild_engine.cpp
    engine/adaptive_optimizer.cpp
    engine/broadphase.cpp
    engine/compression.cpp
    engine/contact_packing.cpp
    engine/convergence.cpp
    engine/escape_search.cpp
    engine/frontier_analyzer.cpp
    engine/free_space_analyzer.cpp
    engine/gap_filling.cpp
    engine/guided_local_search.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/layout_state.cpp
    engine/multi_start_solver.cpp
    engine/overlap_resolver.cpp
    engine/parallel_collision_evaluator.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
    engine/rearrangement.cpp
    engine/region_repack.cpp
    engine/local_region_repack.cpp
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
    engine/aggressive_gap_filler.cpp
    engine/empty_space_map.cpp
    engine/slide_to_contact.cpp
    engine/worker_pool.cpp
)

add_executable(CigerNestingSolver500MaxSpacing0Smoke
    tests/solver_500_max_spacing0_smoke.cpp
    import/importer.cpp
    import/svg_parser.cpp
    import/plt_parser.cpp
    import/dxf_parser.cpp
    geometry/arc.cpp
    geometry/bezier.cpp
    geometry/clearance.cpp
    geometry/flatten.cpp
    geometry/winding.cpp
    geometry/polygon_utils.cpp
    geometry/collision.cpp
    geometry/transformed_shape.cpp
    geometry/spatial_grid.cpp
    engine/acceptance.cpp
    engine/analytic_contact_candidate.cpp
    engine/contact_candidate_provider.cpp
    engine/nfp_cache.cpp
    engine/nfp_candidate_provider.cpp
    engine/inner_fit_candidate_provider.cpp
    engine/adaptive_acceptance.cpp
    engine/destroy_rebuild.cpp
    engine/constructive_rebuild_engine.cpp
    engine/adaptive_optimizer.cpp
    engine/broadphase.cpp
    engine/compression.cpp
    engine/contact_packing.cpp
    engine/convergence.cpp
    engine/escape_search.cpp
    engine/frontier_analyzer.cpp
    engine/free_space_analyzer.cpp
    engine/gap_filling.cpp
    engine/guided_local_search.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/layout_state.cpp
    engine/multi_start_solver.cpp
    engine/overlap_resolver.cpp
    engine/parallel_collision_evaluator.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
    engine/rearrangement.cpp
    engine/region_repack.cpp
    engine/local_region_repack.cpp
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
    engine/aggressive_gap_filler.cpp
    engine/empty_space_map.cpp
    engine/slide_to_contact.cpp
    engine/worker_pool.cpp
)

add_executable(CigerNestingAdaptivePartNeedSmoke
    tests/adaptive_part_need_smoke.cpp
    import/importer.cpp
    import/svg_parser.cpp
    import/plt_parser.cpp
    import/dxf_parser.cpp
    geometry/arc.cpp
    geometry/bezier.cpp
    geometry/clearance.cpp
    geometry/flatten.cpp
    geometry/winding.cpp
    geometry/polygon_utils.cpp
    geometry/collision.cpp
    geometry/transformed_shape.cpp
    geometry/spatial_grid.cpp
    engine/acceptance.cpp
    engine/analytic_contact_candidate.cpp
    engine/contact_candidate_provider.cpp
    engine/nfp_cache.cpp
    engine/nfp_candidate_provider.cpp
    engine/inner_fit_candidate_provider.cpp
    engine/adaptive_acceptance.cpp
    engine/destroy_rebuild.cpp
    engine/constructive_rebuild_engine.cpp
    engine/adaptive_optimizer.cpp
    engine/broadphase.cpp
    engine/compression.cpp
    engine/contact_packing.cpp
    engine/convergence.cpp
    engine/escape_search.cpp
    engine/frontier_analyzer.cpp
    engine/free_space_analyzer.cpp
    engine/gap_filling.cpp
    engine/guided_local_search.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/layout_state.cpp
    engine/multi_start_solver.cpp
    engine/nesting_engine.cpp
    engine/overlap_resolver.cpp
    engine/parallel_collision_evaluator.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
    engine/rearrangement.cpp
    engine/region_repack.cpp
    engine/local_region_repack.cpp
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
    engine/aggressive_gap_filler.cpp
    engine/empty_space_map.cpp
    engine/slide_to_contact.cpp
    engine/worker_pool.cpp
)

add_executable(CigerNestingArchitectureAuditSmoke
    tests/architecture_audit_smoke.cpp
)

add_executable(CigerNestingNoGlobalModeSmoke
    tests/no_global_mode_smoke.cpp
    geometry/clearance.cpp
    geometry/collision.cpp
    geometry/spatial_grid.cpp
    geometry/transformed_shape.cpp
    engine/acceptance.cpp
    engine/analytic_contact_candidate.cpp
    engine/contact_candidate_provider.cpp
    engine/nfp_cache.cpp
    engine/nfp_candidate_provider.cpp
    engine/inner_fit_candidate_provider.cpp
    engine/adaptive_acceptance.cpp
    engine/destroy_rebuild.cpp
    engine/constructive_rebuild_engine.cpp
    engine/adaptive_optimizer.cpp
    engine/broadphase.cpp
    engine/compression.cpp
    engine/contact_packing.cpp
    engine/convergence.cpp
    engine/escape_search.cpp
    engine/frontier_analyzer.cpp
    engine/free_space_analyzer.cpp
    engine/gap_filling.cpp
    engine/guided_local_search.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/layout_state.cpp
    engine/multi_start_solver.cpp
    engine/overlap_resolver.cpp
    engine/parallel_collision_evaluator.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
    engine/rearrangement.cpp
    engine/region_repack.cpp
    engine/local_region_repack.cpp
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
    engine/aggressive_gap_filler.cpp
    engine/empty_space_map.cpp
    engine/slide_to_contact.cpp
    engine/worker_pool.cpp
)

add_executable(CigerNestingNoBBoxShelfSmoke
    tests/no_bbox_shelf_smoke.cpp
    import/importer.cpp
    import/svg_parser.cpp
    import/plt_parser.cpp
    import/dxf_parser.cpp
    geometry/arc.cpp
    geometry/bezier.cpp
    geometry/clearance.cpp
    geometry/flatten.cpp
    geometry/winding.cpp
    geometry/polygon_utils.cpp
    geometry/collision.cpp
    geometry/transformed_shape.cpp
    geometry/spatial_grid.cpp
    engine/acceptance.cpp
    engine/analytic_contact_candidate.cpp
    engine/contact_candidate_provider.cpp
    engine/nfp_cache.cpp
    engine/nfp_candidate_provider.cpp
    engine/inner_fit_candidate_provider.cpp
    engine/adaptive_acceptance.cpp
    engine/destroy_rebuild.cpp
    engine/constructive_rebuild_engine.cpp
    engine/adaptive_optimizer.cpp
    engine/broadphase.cpp
    engine/compression.cpp
    engine/contact_packing.cpp
    engine/convergence.cpp
    engine/escape_search.cpp
    engine/frontier_analyzer.cpp
    engine/free_space_analyzer.cpp
    engine/gap_filling.cpp
    engine/guided_local_search.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/layout_state.cpp
    engine/multi_start_solver.cpp
    engine/overlap_resolver.cpp
    engine/parallel_collision_evaluator.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
    engine/rearrangement.cpp
    engine/region_repack.cpp
    engine/local_region_repack.cpp
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
    engine/aggressive_gap_filler.cpp
    engine/empty_space_map.cpp
    engine/slide_to_contact.cpp
    engine/worker_pool.cpp
)

add_executable(CigerNestingConvergenceSmoke
    tests/convergence_smoke.cpp
    geometry/clearance.cpp
    geometry/collision.cpp
    geometry/spatial_grid.cpp
    geometry/transformed_shape.cpp
    engine/acceptance.cpp
    engine/analytic_contact_candidate.cpp
    engine/contact_candidate_provider.cpp
    engine/nfp_cache.cpp
    engine/nfp_candidate_provider.cpp
    engine/inner_fit_candidate_provider.cpp
    engine/adaptive_acceptance.cpp
    engine/destroy_rebuild.cpp
    engine/constructive_rebuild_engine.cpp
    engine/adaptive_optimizer.cpp
    engine/broadphase.cpp
    engine/compression.cpp
    engine/contact_packing.cpp
    engine/convergence.cpp
    engine/escape_search.cpp
    engine/frontier_analyzer.cpp
    engine/free_space_analyzer.cpp
    engine/gap_filling.cpp
    engine/guided_local_search.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/layout_state.cpp
    engine/multi_start_solver.cpp
    engine/nesting_engine.cpp
    engine/overlap_resolver.cpp
    engine/parallel_collision_evaluator.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
    engine/rearrangement.cpp
    engine/region_repack.cpp
    engine/local_region_repack.cpp
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
    engine/aggressive_gap_filler.cpp
    engine/empty_space_map.cpp
    engine/slide_to_contact.cpp
    engine/worker_pool.cpp
)

add_executable(CigerNestingContourInterlockSmoke
    tests/contour_interlock_smoke.cpp
    geometry/clearance.cpp
    geometry/collision.cpp
    geometry/spatial_grid.cpp
    geometry/transformed_shape.cpp
    engine/acceptance.cpp
    engine/analytic_contact_candidate.cpp
    engine/contact_candidate_provider.cpp
    engine/nfp_cache.cpp
    engine/nfp_candidate_provider.cpp
    engine/inner_fit_candidate_provider.cpp
    engine/adaptive_acceptance.cpp
    engine/destroy_rebuild.cpp
    engine/constructive_rebuild_engine.cpp
    engine/adaptive_optimizer.cpp
    engine/broadphase.cpp
    engine/compression.cpp
    engine/contact_packing.cpp
    engine/convergence.cpp
    engine/escape_search.cpp
    engine/frontier_analyzer.cpp
    engine/free_space_analyzer.cpp
    engine/gap_filling.cpp
    engine/guided_local_search.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/layout_state.cpp
    engine/multi_start_solver.cpp
    engine/overlap_resolver.cpp
    engine/parallel_collision_evaluator.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
    engine/rearrangement.cpp
    engine/region_repack.cpp
    engine/local_region_repack.cpp
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
    engine/aggressive_gap_filler.cpp
    engine/empty_space_map.cpp
    engine/slide_to_contact.cpp
    engine/worker_pool.cpp
)

add_executable(CigerNestingIndustrialQualitySmoke
    tests/industrial_quality_smoke.cpp
    import/importer.cpp
    import/svg_parser.cpp
    import/plt_parser.cpp
    import/dxf_parser.cpp
    geometry/arc.cpp
    geometry/bezier.cpp
    geometry/clearance.cpp
    geometry/flatten.cpp
    geometry/winding.cpp
    geometry/polygon_utils.cpp
    geometry/collision.cpp
    geometry/transformed_shape.cpp
    geometry/spatial_grid.cpp
    engine/acceptance.cpp
    engine/analytic_contact_candidate.cpp
    engine/contact_candidate_provider.cpp
    engine/nfp_cache.cpp
    engine/nfp_candidate_provider.cpp
    engine/inner_fit_candidate_provider.cpp
    engine/adaptive_acceptance.cpp
    engine/destroy_rebuild.cpp
    engine/constructive_rebuild_engine.cpp
    engine/adaptive_optimizer.cpp
    engine/aggressive_gap_filler.cpp
    engine/broadphase.cpp
    engine/compression.cpp
    engine/contact_packing.cpp
    engine/convergence.cpp
    engine/empty_space_map.cpp
    engine/escape_search.cpp
    engine/frontier_analyzer.cpp
    engine/free_space_analyzer.cpp
    engine/gap_filling.cpp
    engine/guided_local_search.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/layout_state.cpp
    engine/multi_start_solver.cpp
    engine/overlap_resolver.cpp
    engine/parallel_collision_evaluator.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
    engine/rearrangement.cpp
    engine/region_repack.cpp
    engine/local_region_repack.cpp
    engine/slide_to_contact.cpp
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
    engine/worker_pool.cpp
)

add_executable(CigerNestingLocalRegionRepackSmoke
    tests/local_region_repack_smoke.cpp
    import/importer.cpp
    import/svg_parser.cpp
    import/plt_parser.cpp
    import/dxf_parser.cpp
    geometry/arc.cpp
    geometry/bezier.cpp
    geometry/clearance.cpp
    geometry/flatten.cpp
    geometry/winding.cpp
    geometry/polygon_utils.cpp
    geometry/collision.cpp
    geometry/transformed_shape.cpp
    geometry/spatial_grid.cpp
    engine/acceptance.cpp
    engine/analytic_contact_candidate.cpp
    engine/contact_candidate_provider.cpp
    engine/nfp_cache.cpp
    engine/nfp_candidate_provider.cpp
    engine/inner_fit_candidate_provider.cpp
    engine/adaptive_acceptance.cpp
    engine/destroy_rebuild.cpp
    engine/constructive_rebuild_engine.cpp
    engine/adaptive_optimizer.cpp
    engine/aggressive_gap_filler.cpp
    engine/broadphase.cpp
    engine/compression.cpp
    engine/contact_packing.cpp
    engine/convergence.cpp
    engine/empty_space_map.cpp
    engine/escape_search.cpp
    engine/frontier_analyzer.cpp
    engine/free_space_analyzer.cpp
    engine/gap_filling.cpp
    engine/guided_local_search.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/layout_state.cpp
    engine/local_region_repack.cpp
    engine/multi_start_solver.cpp
    engine/overlap_resolver.cpp
    engine/parallel_collision_evaluator.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
    engine/rearrangement.cpp
    engine/region_repack.cpp
    engine/slide_to_contact.cpp
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
    engine/worker_pool.cpp
)

add_executable(CigerNestingFitCandidateGenerationSmoke
    tests/fit_candidate_generation_smoke.cpp
    import/importer.cpp
    import/svg_parser.cpp
    import/plt_parser.cpp
    import/dxf_parser.cpp
    geometry/arc.cpp
    geometry/bezier.cpp
    geometry/clearance.cpp
    geometry/flatten.cpp
    geometry/winding.cpp
    geometry/polygon_utils.cpp
    geometry/collision.cpp
    geometry/transformed_shape.cpp
    geometry/spatial_grid.cpp
    engine/acceptance.cpp
    engine/analytic_contact_candidate.cpp
    engine/contact_candidate_provider.cpp
    engine/nfp_cache.cpp
    engine/nfp_candidate_provider.cpp
    engine/inner_fit_candidate_provider.cpp
    engine/adaptive_acceptance.cpp
    engine/destroy_rebuild.cpp
    engine/constructive_rebuild_engine.cpp
    engine/adaptive_optimizer.cpp
    engine/aggressive_gap_filler.cpp
    engine/broadphase.cpp
    engine/compression.cpp
    engine/contact_packing.cpp
    engine/convergence.cpp
    engine/empty_space_map.cpp
    engine/escape_search.cpp
    engine/frontier_analyzer.cpp
    engine/free_space_analyzer.cpp
    engine/gap_filling.cpp
    engine/guided_local_search.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/layout_state.cpp
    engine/local_region_repack.cpp
    engine/multi_start_solver.cpp
    engine/overlap_resolver.cpp
    engine/parallel_collision_evaluator.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
    engine/rearrangement.cpp
    engine/region_repack.cpp
    engine/slide_to_contact.cpp
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
    engine/worker_pool.cpp
)

add_executable(CigerNestingAnalyticContactCandidateSmoke
    tests/analytic_contact_candidate_smoke.cpp
    import/importer.cpp
    import/svg_parser.cpp
    import/plt_parser.cpp
    import/dxf_parser.cpp
    geometry/arc.cpp
    geometry/bezier.cpp
    geometry/clearance.cpp
    geometry/flatten.cpp
    geometry/winding.cpp
    geometry/polygon_utils.cpp
    geometry/collision.cpp
    geometry/transformed_shape.cpp
    geometry/spatial_grid.cpp
    engine/acceptance.cpp
    engine/analytic_contact_candidate.cpp
    engine/contact_candidate_provider.cpp
    engine/nfp_cache.cpp
    engine/nfp_candidate_provider.cpp
    engine/inner_fit_candidate_provider.cpp
    engine/adaptive_acceptance.cpp
    engine/destroy_rebuild.cpp
    engine/constructive_rebuild_engine.cpp
    engine/adaptive_optimizer.cpp
    engine/aggressive_gap_filler.cpp
    engine/broadphase.cpp
    engine/compression.cpp
    engine/contact_packing.cpp
    engine/convergence.cpp
    engine/empty_space_map.cpp
    engine/escape_search.cpp
    engine/frontier_analyzer.cpp
    engine/free_space_analyzer.cpp
    engine/gap_filling.cpp
    engine/guided_local_search.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/layout_state.cpp
    engine/local_region_repack.cpp
    engine/multi_start_solver.cpp
    engine/overlap_resolver.cpp
    engine/parallel_collision_evaluator.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
    engine/rearrangement.cpp
    engine/region_repack.cpp
    engine/slide_to_contact.cpp
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
    engine/worker_pool.cpp
)

add_executable(CigerNestingAcceptanceModelSmoke
    tests/acceptance_model_smoke.cpp
    engine/adaptive_acceptance.cpp
)

add_executable(CigerNestingScoreCalibrationSmoke
    tests/score_calibration_smoke.cpp
    geometry/clearance.cpp
    geometry/collision.cpp
    geometry/spatial_grid.cpp
    geometry/transformed_shape.cpp
    engine/broadphase.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/penalty_system.cpp
)

add_executable(CigerNestingDestroyRebuildSmoke
    tests/destroy_rebuild_smoke.cpp
    import/importer.cpp
    import/svg_parser.cpp
    import/plt_parser.cpp
    import/dxf_parser.cpp
    geometry/arc.cpp
    geometry/bezier.cpp
    geometry/clearance.cpp
    geometry/flatten.cpp
    geometry/winding.cpp
    geometry/polygon_utils.cpp
    geometry/collision.cpp
    geometry/transformed_shape.cpp
    geometry/spatial_grid.cpp
    engine/acceptance.cpp
    engine/adaptive_acceptance.cpp
    engine/analytic_contact_candidate.cpp
    engine/contact_candidate_provider.cpp
    engine/nfp_cache.cpp
    engine/nfp_candidate_provider.cpp
    engine/inner_fit_candidate_provider.cpp
    engine/adaptive_optimizer.cpp
    engine/aggressive_gap_filler.cpp
    engine/broadphase.cpp
    engine/compression.cpp
    engine/contact_packing.cpp
    engine/convergence.cpp
    engine/constructive_rebuild_engine.cpp
    engine/destroy_rebuild.cpp
    engine/empty_space_map.cpp
    engine/escape_search.cpp
    engine/frontier_analyzer.cpp
    engine/free_space_analyzer.cpp
    engine/gap_filling.cpp
    engine/guided_local_search.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/layout_state.cpp
    engine/local_region_repack.cpp
    engine/multi_start_solver.cpp
    engine/overlap_resolver.cpp
    engine/parallel_collision_evaluator.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
    engine/rearrangement.cpp
    engine/region_repack.cpp
    engine/slide_to_contact.cpp
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
    engine/worker_pool.cpp
)

add_executable(CigerNestingConstructiveRebuildSmoke
    tests/constructive_rebuild_smoke.cpp
    import/importer.cpp
    import/svg_parser.cpp
    import/plt_parser.cpp
    import/dxf_parser.cpp
    geometry/arc.cpp
    geometry/bezier.cpp
    geometry/clearance.cpp
    geometry/flatten.cpp
    geometry/winding.cpp
    geometry/polygon_utils.cpp
    geometry/collision.cpp
    geometry/transformed_shape.cpp
    geometry/spatial_grid.cpp
    engine/acceptance.cpp
    engine/adaptive_acceptance.cpp
    engine/analytic_contact_candidate.cpp
    engine/contact_candidate_provider.cpp
    engine/nfp_cache.cpp
    engine/nfp_candidate_provider.cpp
    engine/inner_fit_candidate_provider.cpp
    engine/adaptive_optimizer.cpp
    engine/aggressive_gap_filler.cpp
    engine/broadphase.cpp
    engine/compression.cpp
    engine/contact_packing.cpp
    engine/convergence.cpp
    engine/constructive_rebuild_engine.cpp
    engine/destroy_rebuild.cpp
    engine/empty_space_map.cpp
    engine/escape_search.cpp
    engine/frontier_analyzer.cpp
    engine/free_space_analyzer.cpp
    engine/gap_filling.cpp
    engine/guided_local_search.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/layout_state.cpp
    engine/local_region_repack.cpp
    engine/multi_start_solver.cpp
    engine/overlap_resolver.cpp
    engine/parallel_collision_evaluator.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
    engine/rearrangement.cpp
    engine/region_repack.cpp
    engine/slide_to_contact.cpp
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
    engine/worker_pool.cpp
)

add_executable(CigerNestingEmptyRegionTargetingSmoke
    tests/empty_region_targeting_smoke.cpp
    geometry/clearance.cpp
    geometry/collision.cpp
    geometry/spatial_grid.cpp
    geometry/transformed_shape.cpp
    engine/broadphase.cpp
    engine/empty_space_map.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/penalty_system.cpp
)

add_executable(CigerNestingConvergenceQualitySmoke
    tests/convergence_quality_smoke.cpp
)

add_executable(CigerNestingDirtyRegionRenderSmoke
    tests/dirty_region_render_smoke.cpp
)

add_executable(CigerNestingMultiPartConstructiveQualitySmoke
    tests/multi_part_constructive_quality_smoke.cpp
    import/importer.cpp
    import/svg_parser.cpp
    import/plt_parser.cpp
    import/dxf_parser.cpp
    geometry/arc.cpp
    geometry/bezier.cpp
    geometry/clearance.cpp
    geometry/flatten.cpp
    geometry/winding.cpp
    geometry/polygon_utils.cpp
    geometry/collision.cpp
    geometry/transformed_shape.cpp
    geometry/spatial_grid.cpp
    engine/acceptance.cpp
    engine/adaptive_acceptance.cpp
    engine/analytic_contact_candidate.cpp
    engine/contact_candidate_provider.cpp
    engine/nfp_cache.cpp
    engine/nfp_candidate_provider.cpp
    engine/inner_fit_candidate_provider.cpp
    engine/adaptive_optimizer.cpp
    engine/aggressive_gap_filler.cpp
    engine/broadphase.cpp
    engine/compression.cpp
    engine/contact_packing.cpp
    engine/convergence.cpp
    engine/constructive_rebuild_engine.cpp
    engine/destroy_rebuild.cpp
    engine/empty_space_map.cpp
    engine/escape_search.cpp
    engine/frontier_analyzer.cpp
    engine/free_space_analyzer.cpp
    engine/gap_filling.cpp
    engine/guided_local_search.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/layout_state.cpp
    engine/local_region_repack.cpp
    engine/multi_start_solver.cpp
    engine/overlap_resolver.cpp
    engine/parallel_collision_evaluator.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
    engine/rearrangement.cpp
    engine/region_repack.cpp
    engine/slide_to_contact.cpp
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
    engine/worker_pool.cpp
)

add_executable(CigerNestingPlacementStartSmoke
    tests/placement_start_smoke.cpp
)

add_executable(CigerNestingFakeStatsGuardSmoke
    tests/fake_stats_guard_smoke.cpp
)

add_executable(CigerNestingActiveContactDepthSmoke
    tests/active_contact_depth_smoke.cpp
)

add_executable(CigerNestingBeamSearchQualitySmoke
    tests/beam_search_quality_smoke.cpp
)

add_executable(CigerNestingBenchmarkConsistencySmoke
    tests/benchmark_consistency_smoke.cpp
)

add_executable(CigerNestingRebuildObjectiveSmoke
    tests/rebuild_objective_smoke.cpp
)

add_executable(CigerNestingDestroyBestUpdateSmoke
    tests/destroy_best_update_smoke.cpp
)

add_executable(CigerNestingTemporaryAcceptanceGuardSmoke
    tests/temporary_acceptance_guard_smoke.cpp
)

set(CIGER_CLUSTER_SMOKE_ENGINE_SOURCES
    import/importer.cpp
    import/svg_parser.cpp
    import/plt_parser.cpp
    import/dxf_parser.cpp
    geometry/arc.cpp
    geometry/bezier.cpp
    geometry/clearance.cpp
    geometry/flatten.cpp
    geometry/winding.cpp
    geometry/polygon_utils.cpp
    geometry/collision.cpp
    geometry/transformed_shape.cpp
    geometry/spatial_grid.cpp
    engine/acceptance.cpp
    engine/adaptive_acceptance.cpp
    engine/analytic_contact_candidate.cpp
    engine/contact_candidate_provider.cpp
    engine/nfp_cache.cpp
    engine/nfp_candidate_provider.cpp
    engine/inner_fit_candidate_provider.cpp
    engine/adaptive_optimizer.cpp
    engine/aggressive_gap_filler.cpp
    engine/broadphase.cpp
    engine/compression.cpp
    engine/contact_packing.cpp
    engine/convergence.cpp
    engine/constructive_rebuild_engine.cpp
    engine/destroy_rebuild.cpp
    engine/empty_space_map.cpp
    engine/escape_search.cpp
    engine/frontier_analyzer.cpp
    engine/free_space_analyzer.cpp
    engine/gap_filling.cpp
    engine/guided_local_search.cpp
    engine/layout_eval_cache.cpp
    engine/layout_score.cpp
    engine/layout_score_components.cpp
    engine/layout_state.cpp
    engine/local_region_repack.cpp
    engine/multi_start_solver.cpp
    engine/overlap_resolver.cpp
    engine/parallel_collision_evaluator.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
    engine/rearrangement.cpp
    engine/region_repack.cpp
    engine/slide_to_contact.cpp
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
    engine/worker_pool.cpp
)

add_executable(CigerNestingClusterCompactionSmoke
    tests/cluster_compaction_smoke.cpp
    ${CIGER_CLUSTER_SMOKE_ENGINE_SOURCES}
)

add_executable(CigerNestingCoordinatedClusterMotionSmoke
    tests/coordinated_cluster_motion_smoke.cpp
    ${CIGER_CLUSTER_SMOKE_ENGINE_SOURCES}
)

add_executable(CigerNestingDenseSmallPartCompactionSmoke
    tests/dense_small_part_compaction_smoke.cpp
    ${CIGER_CLUSTER_SMOKE_ENGINE_SOURCES}
)

add_executable(CigerNestingPartialClusterStateSmoke
    tests/partial_cluster_state_smoke.cpp
    ${CIGER_CLUSTER_SMOKE_ENGINE_SOURCES}
)

set(CIGER_NFP_PROVIDER_SMOKE_SOURCES
    geometry/clearance.cpp
    geometry/collision.cpp
    geometry/transformed_shape.cpp
    engine/analytic_contact_candidate.cpp
    engine/contact_candidate_provider.cpp
    engine/nfp_cache.cpp
    engine/nfp_candidate_provider.cpp
    engine/inner_fit_candidate_provider.cpp
)

add_executable(CigerNestingNfpCandidateProviderSmoke
    tests/nfp_candidate_provider_smoke.cpp
    ${CIGER_NFP_PROVIDER_SMOKE_SOURCES}
)

add_executable(CigerNestingInnerFitCandidateProviderSmoke
    tests/inner_fit_candidate_provider_smoke.cpp
    ${CIGER_NFP_PROVIDER_SMOKE_SOURCES}
)

add_executable(CigerNestingNfpCacheSmoke
    tests/nfp_cache_smoke.cpp
    ${CIGER_NFP_PROVIDER_SMOKE_SOURCES}
)

add_executable(CigerNestingHoleAwareNfpSmoke
    tests/hole_aware_nfp_smoke.cpp
    ${CIGER_NFP_PROVIDER_SMOKE_SOURCES}
)

add_executable(CigerNestingContourLockingNfpSmoke
    tests/contour_locking_nfp_smoke.cpp
    ${CIGER_NFP_PROVIDER_SMOKE_SOURCES}
)

add_executable(CigerNestingNfpConstructiveRebuildSmoke
    tests/nfp_constructive_rebuild_smoke.cpp
    ${CIGER_CLUSTER_SMOKE_ENGINE_SOURCES}
)

add_executable(CigerNestingDenseNfpPackingSmoke
    tests/dense_nfp_packing_smoke.cpp
    ${CIGER_CLUSTER_SMOKE_ENGINE_SOURCES}
)

add_executable(CigerNestingClusterReinsertQualitySmoke
    tests/cluster_reinsert_quality_smoke.cpp
)

target_include_directories(CigerNestingGeometryCollisionSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingGeometryCollisionSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingSolverQualitySmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingSolverQualitySmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingUltraRefinementSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingUltraRefinementSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingClearanceSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingClearanceSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingPerformanceStressSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingPerformanceStressSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingDeltaScoreSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingDeltaScoreSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingMultiDeltaSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingMultiDeltaSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingGapFillingSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingGapFillingSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingRearrangementSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingRearrangementSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingEscapeSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingEscapeSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingBenchmarkRunner PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingBenchmarkRunner PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingLargeScaleQualitySmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingLargeScaleQualitySmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingSolver500MaxSpacing0Smoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingSolver500MaxSpacing0Smoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingAdaptivePartNeedSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingAdaptivePartNeedSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingArchitectureAuditSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingArchitectureAuditSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingNoGlobalModeSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingNoGlobalModeSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingNoBBoxShelfSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingNoBBoxShelfSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingConvergenceSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingConvergenceSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingContourInterlockSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingContourInterlockSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingIndustrialQualitySmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingIndustrialQualitySmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingLocalRegionRepackSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingLocalRegionRepackSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingFitCandidateGenerationSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingFitCandidateGenerationSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingAnalyticContactCandidateSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingAnalyticContactCandidateSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingAcceptanceModelSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingAcceptanceModelSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingScoreCalibrationSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingScoreCalibrationSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingDestroyRebuildSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingDestroyRebuildSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingConstructiveRebuildSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingConstructiveRebuildSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingEmptyRegionTargetingSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingEmptyRegionTargetingSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingConvergenceQualitySmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingConvergenceQualitySmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingDirtyRegionRenderSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingDirtyRegionRenderSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingMultiPartConstructiveQualitySmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingMultiPartConstructiveQualitySmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingPlacementStartSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingPlacementStartSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingFakeStatsGuardSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingFakeStatsGuardSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingActiveContactDepthSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingActiveContactDepthSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingBeamSearchQualitySmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingBeamSearchQualitySmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingBenchmarkConsistencySmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingBenchmarkConsistencySmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingRebuildObjectiveSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingRebuildObjectiveSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingDestroyBestUpdateSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingDestroyBestUpdateSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingTemporaryAcceptanceGuardSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingTemporaryAcceptanceGuardSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingClusterCompactionSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingClusterCompactionSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingCoordinatedClusterMotionSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingCoordinatedClusterMotionSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingDenseSmallPartCompactionSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingDenseSmallPartCompactionSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingPartialClusterStateSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingPartialClusterStateSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingNfpCandidateProviderSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingNfpCandidateProviderSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingInnerFitCandidateProviderSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingInnerFitCandidateProviderSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingNfpCacheSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingNfpCacheSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingHoleAwareNfpSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingHoleAwareNfpSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingContourLockingNfpSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingContourLockingNfpSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingNfpConstructiveRebuildSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingNfpConstructiveRebuildSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingDenseNfpPackingSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingDenseNfpPackingSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingClusterReinsertQualitySmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingClusterReinsertQualitySmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)

if(MSVC)
    target_compile_options(CigerNesting PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingGeometryCollisionSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingSolverQualitySmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingUltraRefinementSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingClearanceSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingPerformanceStressSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingDeltaScoreSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingMultiDeltaSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingGapFillingSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingRearrangementSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingEscapeSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingBenchmarkRunner PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingLargeScaleQualitySmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingSolver500MaxSpacing0Smoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingAdaptivePartNeedSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingArchitectureAuditSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingNoGlobalModeSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingNoBBoxShelfSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingConvergenceSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingContourInterlockSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingIndustrialQualitySmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingLocalRegionRepackSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingFitCandidateGenerationSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingAnalyticContactCandidateSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingAcceptanceModelSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingScoreCalibrationSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingDestroyRebuildSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingConstructiveRebuildSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingEmptyRegionTargetingSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingConvergenceQualitySmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingDirtyRegionRenderSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingMultiPartConstructiveQualitySmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingPlacementStartSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingFakeStatsGuardSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingActiveContactDepthSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingBeamSearchQualitySmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingBenchmarkConsistencySmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingRebuildObjectiveSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingDestroyBestUpdateSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingTemporaryAcceptanceGuardSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingClusterCompactionSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingCoordinatedClusterMotionSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingDenseSmallPartCompactionSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingPartialClusterStateSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingNfpCandidateProviderSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingInnerFitCandidateProviderSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingNfpCacheSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingHoleAwareNfpSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingContourLockingNfpSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingNfpConstructiveRebuildSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingDenseNfpPackingSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingClusterReinsertQualitySmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
else()
    target_compile_options(CigerNesting PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingGeometryCollisionSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingSolverQualitySmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingUltraRefinementSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingClearanceSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingPerformanceStressSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingDeltaScoreSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingMultiDeltaSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingGapFillingSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingRearrangementSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingEscapeSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingBenchmarkRunner PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingLargeScaleQualitySmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingSolver500MaxSpacing0Smoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingAdaptivePartNeedSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingArchitectureAuditSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingNoGlobalModeSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingNoBBoxShelfSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingConvergenceSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingContourInterlockSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingIndustrialQualitySmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingLocalRegionRepackSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingFitCandidateGenerationSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingAnalyticContactCandidateSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingAcceptanceModelSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingScoreCalibrationSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingDestroyRebuildSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingConstructiveRebuildSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingEmptyRegionTargetingSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingConvergenceQualitySmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingDirtyRegionRenderSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingMultiPartConstructiveQualitySmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingPlacementStartSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingFakeStatsGuardSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingActiveContactDepthSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingBeamSearchQualitySmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingBenchmarkConsistencySmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingRebuildObjectiveSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingDestroyBestUpdateSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingTemporaryAcceptanceGuardSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingClusterCompactionSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingCoordinatedClusterMotionSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingDenseSmallPartCompactionSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingPartialClusterStateSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingNfpCandidateProviderSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingInnerFitCandidateProviderSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingNfpCacheSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingHoleAwareNfpSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingContourLockingNfpSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingNfpConstructiveRebuildSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingDenseNfpPackingSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingClusterReinsertQualitySmoke PRIVATE -Wall -Wextra -Wpedantic)
endif()




