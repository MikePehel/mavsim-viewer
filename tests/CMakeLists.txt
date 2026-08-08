set(FIXTURES_DIR ${CMAKE_SOURCE_DIR}/tests/fixtures)

# Tests use assert() for both setup and checks — ensure NDEBUG is never set
# so asserts are always active, even in Release builds.
# remove_definitions is a no-op on per-target flags added by CMake, so we
# strip them from the Release flags directly.
if(MSVC)
    string(REPLACE "/DNDEBUG" "" CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
    string(REPLACE "/DNDEBUG" "" CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO}")
else()
    string(REPLACE "-DNDEBUG" "" CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
    string(REPLACE "-DNDEBUG" "" CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO}")
endif()

# ULog parser tests
add_executable(test_ulog_parser test_ulog_parser.c)
target_link_libraries(test_ulog_parser PRIVATE hawkeye-core)
target_compile_definitions(test_ulog_parser PRIVATE FIXTURES_DIR="${FIXTURES_DIR}")
add_test(NAME ulog_parser COMMAND test_ulog_parser)

# ULog replay tests
add_executable(test_ulog_replay test_ulog_replay.c)
target_link_libraries(test_ulog_replay PRIVATE hawkeye-core)
target_compile_definitions(test_ulog_replay PRIVATE FIXTURES_DIR="${FIXTURES_DIR}")
add_test(NAME ulog_replay COMMAND test_ulog_replay)

# Data source integration tests
add_executable(test_data_source test_data_source.c)
target_link_libraries(test_data_source PRIVATE hawkeye-core)
target_compile_definitions(test_data_source PRIVATE FIXTURES_DIR="${FIXTURES_DIR}")
add_test(NAME data_source COMMAND test_data_source)

# Algorithm tests (CUSUM takeoff detection, Pearson correlation, home tier)
add_executable(test_algorithms test_algorithms.c)
target_link_libraries(test_algorithms PRIVATE hawkeye-core)
target_compile_definitions(test_algorithms PRIVATE FIXTURES_DIR="${FIXTURES_DIR}")
add_test(NAME algorithms COMMAND test_algorithms)

# UI logic tests (vehicle selection, edge-indicator math — no Raylib needed)
add_executable(test_ui_logic test_ui_logic.c)
target_include_directories(test_ui_logic PRIVATE ${CMAKE_SOURCE_DIR}/src)
if(UNIX AND NOT APPLE)
    target_link_libraries(test_ui_logic PRIVATE m)
endif()
add_test(NAME ui_logic COMMAND test_ui_logic)

# OBJ model validation (catches missing texcoords that crash Raylib)
add_executable(test_obj_models test_obj_models.c)
target_compile_definitions(test_obj_models PRIVATE MODELS_DIR="${CMAKE_SOURCE_DIR}/models")
add_test(NAME obj_models COMMAND test_obj_models)

# Theme parser + trail LOD tests (need raylib headers for Color/Vector3)
if(TARGET raylib)
    add_executable(test_theme test_theme.c ${CMAKE_SOURCE_DIR}/src/theme.c)
    target_include_directories(test_theme PRIVATE ${CMAKE_SOURCE_DIR}/src)
    target_link_libraries(test_theme PRIVATE raylib)
    target_compile_definitions(test_theme PRIVATE FIXTURES_DIR="${FIXTURES_DIR}")
    add_test(NAME theme COMMAND test_theme)
endif()

# Terrain parameter extraction tests. Compiled standalone (does NOT
# link hawkeye-core) so the test runs before the vendored terrain library is
# wired into the main build: the test provides its own stub for
# terrain_set_params(). When the terrain library lands and hawkeye-core picks
# up these sources, this test will still build standalone — the stub takes
# precedence at link time within this executable.
set(TERRAIN_FIXTURES_DIR ${CMAKE_SOURCE_DIR}/tests/terrain/fixtures)
add_executable(test_ulog_extract
    terrain/test_ulog_extract.c
    ${CMAKE_SOURCE_DIR}/src/ulog/terrain_params_extract.c
    ${CMAKE_SOURCE_DIR}/src/terrain/terrain_params.c
)
target_include_directories(test_ulog_extract PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_compile_definitions(test_ulog_extract PRIVATE
    FIXTURES_DIR="${FIXTURES_DIR}"
    TERRAIN_FIXTURES_DIR="${TERRAIN_FIXTURES_DIR}"
)
add_test(NAME ulog_extract COMMAND test_ulog_extract)

# Live-SIH MAVLink listener tests. Same standalone pattern as
# test_ulog_extract: compiles the listener + apply skeleton directly with
# a local terrain_set_params() stub, so the test runs independently of
# the vendored terrain library being wired into hawkeye-core. Needs the
# MAVLink common dialect on the include path for mavlink_param_value_t.
add_executable(test_mavlink_listener
    terrain/test_mavlink_listener.c
    ${CMAKE_SOURCE_DIR}/src/mavlink/terrain_params_listener.c
    ${CMAKE_SOURCE_DIR}/src/terrain/terrain_params.c
)
target_include_directories(test_mavlink_listener PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${MAVLINK_DIR}
    ${MAVLINK_DIR}/common
)
target_compile_definitions(test_mavlink_listener PRIVATE
    MAVLINK_COMM_NUM_BUFFERS=16
)
add_test(NAME mavlink_listener COMMAND test_mavlink_listener)
