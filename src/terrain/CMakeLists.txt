# =============================================================================
# Hawkeye-side build of the vendored PX4 terrain library.
#
# `terrain.c` / `terrain.h` are physically shared with PX4 firmware (see the
# banner comments at the top of each file). This CMakeLists builds them as a
# static library `hawkeye-terrain` that is consumable from:
#
#   - the native Hawkeye build (`add_subdirectory(src/terrain)` from the root
#     CMakeLists when the renderer or parameter-ingestion side wires it in)
#   - the WASM Hawkeye build (`add_subdirectory(${HAWKEYE_ROOT}/src/terrain
#     hawkeye-terrain)` from `wasm/CMakeLists.txt` once the renderer
#     side wires `terrain()` into the per-vertex evaluation path)
#
# When configured *standalone* (e.g. `cmake -S src/terrain -B build-terrain`),
# it additionally builds the determinism harness in `tests/terrain/`. This
# is the verification path used by both the native (host) determinism test
# and the WASM (browser) determinism test:
#
#   Native:
#     cmake -S src/terrain -B src/terrain/build-native
#     cmake --build src/terrain/build-native -j
#     ./src/terrain/build-native/determinism_test         # verify
#     ./src/terrain/build-native/determinism_test --generate \
#         > tests/terrain/determinism_reference.h        # regenerate ref
#
#   WASM (requires emsdk in PATH):
#     emcmake cmake -S src/terrain -B src/terrain/build-wasm
#     cmake --build src/terrain/build-wasm -j
#     (cd src/terrain/build-wasm && python3 -m http.server 8000)
#     # open http://localhost:8000/determinism_test.html
#
# Build flags: strict C99, -Wall -Wextra -Wpedantic, zero warnings expected.
# Anything else is a portability bug in the shared source — report upstream
# rather than patching around it locally.
# =============================================================================

cmake_minimum_required(VERSION 3.14)
project(hawkeye-terrain C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)        # no GNU extensions — keep portable

# -----------------------------------------------------------------------------
# Static library: the shared terrain source compiled into a linkable archive.
# -----------------------------------------------------------------------------
add_library(hawkeye-terrain STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/terrain.c
    # SDF library (vendored from PX4 src/lib/terrain_sdf/) —
    # same shared-source contract as terrain.c.
    #
    # scene.c uses #include "../terrain/terrain.h" (hash3d,
    # hash_to_unit_float, terrain_get_seed), so the relative path
    # requires terrain_sdf/ to be a sibling of terrain/, matching the
    # PX4 layout.
    #
    # brought in terrain_sdf.c (sphere tracer + box SDF with
    # yaw rotation) — Hawkeye doesn't ray-march beams from this code,
    # but the file's static state (sdf_scene_add etc.) is part of the
    # vendored contract and benefits from the same -Werror portability
    # gates as terrain.c.
    ${CMAKE_CURRENT_SOURCE_DIR}/../terrain_sdf/terrain_sdf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../terrain_sdf/scene.c
)

target_include_directories(hawkeye-terrain PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    # Expose src/terrain_sdf/ so downstream targets that want
    # `#include "terrain_sdf/scene.h"` from src/ root can resolve it,
    # and scene.c's `#include "scene.h"` finds its own header.
    ${CMAKE_CURRENT_SOURCE_DIR}/../terrain_sdf
)

# Strict portability — match what the upstream PX4 build enforces on the
# shared source. Any warning here is a portability issue worth investigating.
if(MSVC)
    target_compile_options(hawkeye-terrain PRIVATE /W4 /WX)
else()
    target_compile_options(hawkeye-terrain PRIVATE
        -Wall -Wextra -Wpedantic
        -Werror

        # -ffp-contract=off disables compiler-emitted fused multiply-add.
        #
        # Apple Silicon (and other ARM-A profile) Clang fuses `a*b + c` into
        # a single FMA instruction by default; emcc-wasm32 baseline does NOT
        # have FMA (it is a SIMD extension, not always enabled); the
        # firmware target (Cortex-M7) has no FPU FMA either. Without this
        # flag, host-native and wasm32 produce different fBm bit patterns
        # — visually invisible (drift << 1 mm at terrain scale) but enough
        # to push the determinism harness past its byte-equal budget on
        # the float bucket. With this flag the three targets agree.
        #
        # On MSVC the equivalent is /fp:precise which is the default, so
        # the MSVC branch above does not need a corresponding clause.
        -ffp-contract=off
    )
endif()

# libm on platforms that need an explicit link (not Apple, not Emscripten).
if(UNIX AND NOT APPLE AND NOT EMSCRIPTEN)
    target_link_libraries(hawkeye-terrain PUBLIC m)
endif()

# -----------------------------------------------------------------------------
# Determinism harness — only built when this CMakeLists is the top-level
# project. When pulled in via add_subdirectory() from the host Hawkeye
# build, the harness is skipped (the host has its own ctest invocation
# strategy that the integration engineer wires up).
# -----------------------------------------------------------------------------
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(HAWKEYE_TERRAIN_TESTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../tests/terrain)

    if(EMSCRIPTEN)
        # -------------------------------------------------------------------
        # WASM target: emit .js + .wasm; serve next to a hand-written HTML
        # harness that loads the module, runs the 100 fixtures, and writes
        # the pass/fail + max-ULP report into the page DOM.
        # -------------------------------------------------------------------
        add_executable(determinism_test
            ${HAWKEYE_TERRAIN_TESTS_DIR}/determinism_test.c
        )
        target_link_libraries(determinism_test PRIVATE hawkeye-terrain)
        target_compile_definitions(determinism_test PRIVATE
            DETERMINISM_TEST_WASM=1
        )
        set_target_properties(determinism_test PROPERTIES SUFFIX ".js")
        target_link_options(determinism_test PRIVATE
            "-sMODULARIZE=1"
            "-sEXPORT_ES6=1"
            "-sEXPORT_NAME=createDeterminismTest"
            "-sEXPORTED_FUNCTIONS=['_main','_determinism_run_into_buffer','_determinism_num_inputs','_determinism_num_hash_exact','_terrain','_terrain_set_params','_terrain_benchmark','_malloc','_free']"
            "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','HEAPU8','HEAPU32','HEAPF32','UTF8ToString']"
            "-sINITIAL_MEMORY=4MB"
            # web for the HTML harness; node for headless CI / smoke runs.
            "-sENVIRONMENT=web,node"
        )

        # Copy the HTML harness next to the build artifacts.
        add_custom_command(TARGET determinism_test POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${HAWKEYE_TERRAIN_TESTS_DIR}/determinism_test.html
                $<TARGET_FILE_DIR:determinism_test>/determinism_test.html
            COMMAND ${CMAKE_COMMAND} -E echo
                "[hawkeye-terrain] WASM harness built. Serve build dir with python3 -m http.server and open determinism_test.html"
        )
    else()
        # -------------------------------------------------------------------
        # Native target: a CLI test binary. Run with no args to verify
        # against the committed reference vectors. Run with --generate to
        # emit a fresh tests/terrain/determinism_reference.h to stdout.
        # -------------------------------------------------------------------
        enable_testing()
        add_executable(determinism_test
            ${HAWKEYE_TERRAIN_TESTS_DIR}/determinism_test.c
        )
        target_link_libraries(determinism_test PRIVATE hawkeye-terrain)
        add_test(NAME terrain_determinism COMMAND determinism_test)
    endif()
endif()
