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
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
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
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
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
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
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
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
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
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
    engine/worker_pool.cpp
)

add_executable(CigerNestingConvergenceSmoke
    tests/convergence_smoke.cpp
    geometry/clearance.cpp
    geometry/collision.cpp
    geometry/spatial_grid.cpp
    geometry/transformed_shape.cpp
    engine/acceptance.cpp
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
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
    engine/worker_pool.cpp
)

add_executable(CigerNestingContourInterlockSmoke
    tests/contour_interlock_smoke.cpp
    geometry/clearance.cpp
    geometry/collision.cpp
    geometry/spatial_grid.cpp
    geometry/transformed_shape.cpp
    engine/acceptance.cpp
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
    engine/small_part_filler.cpp
    engine/tabu_memory.cpp
    engine/ultra_refinement.cpp
    engine/worker_pool.cpp
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
target_include_directories(CigerNestingConvergenceSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingConvergenceSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingContourInterlockSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingContourInterlockSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)

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
    target_compile_options(CigerNestingConvergenceSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingContourInterlockSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
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
    target_compile_options(CigerNestingConvergenceSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingContourInterlockSmoke PRIVATE -Wall -Wextra -Wpedantic)
endif()
