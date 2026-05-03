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

if(MSVC)
    target_compile_options(CigerNesting PRIVATE /W4 /permissive- /EHsc /utf-8)
else()
    target_compile_options(CigerNesting PRIVATE -Wall -Wextra -Wpedantic)
endif()
