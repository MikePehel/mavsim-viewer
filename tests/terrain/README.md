# Terrain test harness

Fixtures and integration tooling for the SIH terrain pipeline: PX4 writes
`SIH_TERR_*` parameters, Hawkeye ingests them from a ULog or a live MAVLink
link, and the renderer draws the matching surface.

## Scope

| Lives here | Lives elsewhere |
|---|---|
| SIH-terrain replay fixtures (`fixtures/*.ulg`) | Pre-terrain ULog fixtures: `../fixtures/*.ulg` |
| Integration smoke entry point (`integration_smoke.py`) | ULog parser unit tests: `../test_ulog_parser.c` |
| Visual regression baselines (`baselines/`) | WASM cross-browser matrix: `../../wasm/TESTING.md` |
| Determinism harness (`determinism_test.*`) | Native unit-test harness: `../CMakeLists.txt` |

## Integration smoke

`integration_smoke.py` covers everything in the replay → render pipeline that
is verifiable without a GUI or display server:

```
python3 tests/terrain/integration_smoke.py
```

Asserts:

1. `fixtures/sih_terrain_on.ulg` exposes the expected `SIH_TERR_*` values —
   defaults at boot, then `EN=1` / `AMP=20` / `SEED=3` / `MPC_ALT_MODE=1` at
   t≈7.1 s.
2. The ULog and MAVLink unit tests (`ulog_extract`, `mavlink_listener`) pass
   in the current `build/` tree.
3. `build/hawkeye` has the orchestration symbols linked:
   `hawkeye_terrain_set_apply_callback`, `terrain_renderer_apply_params`,
   `terrain_renderer_set_drone_velocity_ned`,
   `terrain_renderer_{init,draw,shutdown}`, `terrain_set_params`.
4. The ingestion → renderer chain is wired: `src/main.c` registers
   `terrain_renderer_apply_params` as the apply callback, so both ingestion
   paths fan out through the callback rather than calling the renderer
   directly; `src/data_source_ulog.c` applies on log load; and
   `src/mavlink_receiver.c` routes `PARAM_VALUE` into the listener.

Not covered: that terrain visibly renders on screen. That needs a real window
— see `baselines/README.md`.

## Fixtures

### `fixtures/sih_terrain_on.ulg`

PX4 SITL `sihsim_quadx` flight, ~218 s, flown as `AUTO.MISSION` at 30 m AGL
with `MPC_ALT_MODE=1` (terrain-follow).

Boot parameters are airframe defaults with terrain disabled. Runtime
`PARAM_SET`s land in `changed_parameters` at t≈7.1 s:

| Param | initial | changed @ 7.1 s | Notes |
|---|---|---|---|
| `SIH_TERR_EN` | 0 | 1 (`0x3F800000`) | int32-typed, float-encoded value |
| `SIH_TERR_AMP` | 0.0 | 20.0 | float32, metres |
| `SIH_TERR_SEED` | 0 | 3 (`0x40400000`) | int32-typed, float-encoded value |
| `SIH_TERR_FREQ` | 0.005 | — | unchanged |
| `SIH_TERR_OCT` | 6 | — | unchanged |
| `SIH_TERR_HURST` | 0.7 | — | unchanged |
| `SIH_TERR_EROSION` | 1.0 | — | unchanged |
| `MPC_ALT_MODE` | 2 | 1 (`0x3F800000`) | terrain-follow on |
| `SIH_LOC_LAT0` / `SIH_LOC_LON0` | 47.397743 / 8.545594 | — | sihsim default origin |

PX4's `PARAM_SET` wire-encodes every value as a float, so int32-typed params
arrive as the IEEE-754 bit pattern of the integer. Anything reading
`SIH_TERR_*` must reapply the float-bits→int cast — see
`src/ulog/terrain_params_extract.c`.

Terrain turns on 7.1 s into the recording, so a test wanting the terrain-on
state must either replay past t=7.1 s before sampling, or merge
`initial_parameters` with `changed_parameters` before applying.

The fixture is kept whole rather than trimmed to the parameter section,
because it also serves as the replay source for attended visual capture.

### Regenerating

Fly any `sihsim_quadx` SITL mission with `SIH_TERR_EN=1` and a non-zero
`SIH_TERR_AMP`, then copy the resulting `.ulg` out of the PX4 log directory
(`build/px4_sitl_default/rootfs/log/<date>/<time>.ulg`). Copy only the
scenarios you need — full flight logs are large and most variants are
redundant for terrain coverage.

## Building and running

```sh
# Native
git submodule update --init --recursive
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build --config Release -j10
ctest --test-dir build --output-on-failure -C Release

# WASM (requires emsdk on PATH)
emcmake cmake -S wasm -B wasm/build -DCMAKE_BUILD_TYPE=Release
cmake --build wasm/build -j10
```

## Frame-rate contract

FPS is not measured by the automated harness — Hawkeye is a GUI application
and there is no headless rendering path. The targets, checked during attended
capture, are:

- chase camera, WebGL2/WASM: ≥ 30 FPS
- ortho view, full rings: ≥ 30 FPS

Procedure: build WASM Release, serve `wasm/build/` over HTTP, load the
fixture, seek past t=7.1 s, park the camera at the reference pose from
`baselines/README.md`, and average `performance.now()` frame times over a
10 s window for each view.
