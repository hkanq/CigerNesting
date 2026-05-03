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
    geometry/flatten.cpp
    geometry/winding.cpp
    geometry/polygon_utils.cpp
    geometry/collision.cpp
    geometry/transformed_shape.cpp
    geometry/spatial_grid.cpp
    engine/broadphase.cpp
    engine/compression.cpp
    engine/guided_local_search.cpp
    engine/layout_score.cpp
    engine/layout_state.cpp
    engine/multi_start_solver.cpp
    engine/overlap_resolver.cpp
    engine/parallel_collision_evaluator.cpp
    engine/penalty_system.cpp
    engine/pose_sampler.cpp
    engine/worker_pool.cpp
)

target_include_directories(CigerNestingGeometryCollisionSmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingGeometryCollisionSmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_include_directories(CigerNestingSolverQualitySmoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(CigerNestingSolverQualitySmoke PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)

if(MSVC)
    target_compile_options(CigerNesting PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingGeometryCollisionSmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_options(CigerNestingSolverQualitySmoke PRIVATE /W4 /permissive- /EHsc /utf-8)
else()
    target_compile_options(CigerNesting PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingGeometryCollisionSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(CigerNestingSolverQualitySmoke PRIVATE -Wall -Wextra -Wpedantic)
endif()
