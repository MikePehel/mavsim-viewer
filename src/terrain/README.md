# Hawkeye terrain library — viewer-side build of the vendored PX4 source

This directory holds the **shared** procedural terrain library — the same C
source the SIH firmware compiles into PX4. Both sides build the same bytes;
that is what makes "the viewer renders the surface the drone flew over" a
tautology rather than a hope.

## Provenance

| File | Source | Upstream commit |
|---|---|---|
| `terrain.c` | `PX4-Autopilot/src/lib/terrain/terrain.c` | `eecb4cb490` |
| `terrain.h` | `PX4-Autopilot/src/lib/terrain/terrain.h` | `eecb4cb490` |
| `../terrain_sdf/terrain_sdf.h` | `PX4-Autopilot/src/lib/terrain_sdf/terrain_sdf.h` | `eecb4cb490` |
| `../terrain_sdf/terrain_sdf.c` | `PX4-Autopilot/src/lib/terrain_sdf/terrain_sdf.c` | `eecb4cb490` |
| `../terrain_sdf/scene.h` | `PX4-Autopilot/src/lib/terrain_sdf/scene.h` | `eecb4cb490` |
| `../terrain_sdf/scene.c` | `PX4-Autopilot/src/lib/terrain_sdf/scene.c` | `eecb4cb490` |

**Pin history** (do not bump backwards):

| SHA | What changed |
|---|---|
| `c13a35382d` | Original API freeze |
| `ec15322aa0` | SDF raycaster pivot: `s_params.max_slope` removed, Lipschitz-bound consumer moved to the renderer |
| `6c1e3da4a1` | Lattice walls replace the heightfield: walls placed on a cell lattice via `scene_wall_at_cell(i, j)` keyed off `SIH_TERR_SEED`; `sdf_prim_t` gains `cos_yaw`/`sin_yaw`; `terrain_set_params()` shrinks to 4 args |
| `a78b5c0981` | fBm elevation restored alongside the walls; `terrain_set_params()` grows back to 6 args `(amp, wavelength, seed, home_n, home_e, plane_deg)` |
| **`eecb4cb490`** | **Current.** Wall lattice evaluated as an on-demand SDF field |

**Do NOT vendor** these intermediate / superseded SHAs:
- `472521144a` — static-walls initial ship; lacks per-seed variation
- `7f0840f526` — procedural restoration with a short `[2, 8) m` height range, superseded by the tall-walls fix
- `4235eaab7b` — procedural + tall walls; the "fBm and walls in parallel" design was retracted by the lattice redesign

The vendored files carry a `VENDORED FROM PX4-Autopilot` banner immediately
after the BSD-3 licence header. The body of each file is **byte-identical**
to the upstream copy at the pinned commit — verified by

```
diff <(sed -n '/#ifndef PX4_SRC_LIB_TERRAIN_TERRAIN_H_/,$p' src/terrain/terrain.h) \
     <(sed -n '/#ifndef PX4_SRC_LIB_TERRAIN_TERRAIN_H_/,$p' $PX4_ROOT/src/lib/terrain/terrain.h)

diff <(sed -n '/^#include "terrain.h"/,$p' src/terrain/terrain.c) \
     <(sed -n '/^#include "terrain.h"/,$p' $PX4_ROOT/src/lib/terrain/terrain.c)
```

## Re-syncing from upstream

The shared source is **NOT** meant to be modified locally. If you need a
change, post an upstream API Change Proposal and
wait for both sides to agree. The viewer must not get ahead of the
firmware on this surface.

When the upstream copy moves forward and the changes are API-additive
only:

```bash
cp $PX4_ROOT/src/lib/terrain/terrain.{c,h} src/terrain/
# Re-add the VENDORED banner at the top of each file (preserves provenance).
# Bump the "Upstream commit" hash in the banner to the new HEAD.
cmake --build src/terrain/build-native
./src/terrain/build-native/determinism_test --generate \
    > tests/terrain/determinism_reference.h
cmake --build src/terrain/build-native
./src/terrain/build-native/determinism_test            # must PASS
```

If the upstream copy changes the noise math itself (intentional), the
reference vector regeneration is expected; verify visually that the
resulting surface still matches what the firmware produces. If the diff
is *unintentional*, that is a regression — block the sync and ping the
SIH side.

## Building

### Native (host C99 toolchain)

```bash
cmake -S src/terrain -B src/terrain/build-native
cmake --build src/terrain/build-native -j
./src/terrain/build-native/determinism_test            # verify
```

Compiles `terrain.c` as `libhawkeye-terrain.a` and links the determinism
harness against it. Strict flags: `-std=c99 -Wall -Wextra -Wpedantic
-Werror`, zero warnings expected. Anything that fires here is a
portability bug in the shared source and should be reported upstream
rather than patched around locally.

### WASM (emsdk toolchain)

```bash
source $EMSDK_ROOT/emsdk_env.sh
emcmake cmake -S src/terrain -B src/terrain/build-wasm
cmake --build src/terrain/build-wasm -j
(cd src/terrain/build-wasm && python3 -m http.server 8000)
# open http://localhost:8000/determinism_test.html
```

Emits `determinism_test.{js,wasm,html}` into `src/terrain/build-wasm/`. The
HTML harness is the cross-platform verification: it loads the wasm32
module and compares its `terrain()` output to the reference bit patterns
captured on the native host (embedded in the page as a `Uint32Array`).
Verdict appears at the top of the page; machine-readable result is
exposed as `window.__determinism_result` for headless CI scraping.

### Wiring into the host Hawkeye build

This `CMakeLists.txt` is `add_subdirectory()`-friendly. The native and
WASM Hawkeye builds can pull the library in via:

```cmake
add_subdirectory(${CMAKE_SOURCE_DIR}/src/terrain hawkeye-terrain)
target_link_libraries(hawkeye PRIVATE hawkeye-terrain)
```

When pulled in as a subdirectory the determinism harness is **skipped**
(only enabled when this `CMakeLists.txt` is the top-level project). The
integration owner wires the determinism test into the host `ctest`
invocation separately.

## Frame contract (recap)

This library is unconditionally NED:

- `terrain(north_m, east_m)` — North/East metres from the home reference.
- Height return value positive up.
- By construction `terrain(home_N, home_E) == 0` after
  `terrain_set_params(..., home_n, home_e)`.

The viewer (raylib, Y-up) is responsible for applying the
`(N, E, D) -> (E, -D, N)` transform **before** calling this function. The
library itself knows nothing about Y-up — that is intentional, so both
sides see exactly the same world.

## Determinism contract

Two buckets, two tolerance levels (see `tests/terrain/determinism_test.c`):

| Bucket | Inputs | Tolerance | Why |
|---|---|---|---|
| `HASH-EXACT` | 50 cases at integer (N, E), 1 octave, `wavelength=1`, `erosion=0` | **bit-equal** required | reduces to one `hash2d` + `hash_to_float`; integer hash is bit-exact across compilers by construction |
| `FBM-DRIFT` | 50 cases with realistic SIH params, multi-octave, fractional (N, E) | **`<= 64` ULP** | accumulation order + FPU rounding produce 1..N ULP between architectures; the budget catches any systemic divergence (e.g. leaked `-ffast-math`) |

Native self-consistency:

| Metric | Value |
|---|---|
| host triple | `aarch64-apple-darwin25.3.0`, AppleClang 17.0.0 |
| hash-exact | 50/50 PASS, 0 mismatches |
| fbm-drift | 50/50 PASS, worst ULP = 0 (self-comparison) |

## Host vs WASM consistency report

| Metric | Native (host) | WASM (Node loader) |
|---|---|---|
| toolchain | `aarch64-apple-darwin25.3.0`, AppleClang 17.0.0 | emcc 5.0.6, wasm32-unknown-emscripten |
| hash-exact bucket | 50/50 PASS | 50/50 PASS |
| fbm-drift bucket | 50/50 PASS | 50/50 PASS |
| worst ULP vs committed reference | 0 | **0** |

**The wasm32 build is bit-equal to the native host build for all 100
fixtures** — including the float fBm accumulator that the Shared Source
Contract budgets for ~1 ULP of drift. No drift was observed.

### Why zero drift (and what was almost wrong)

First-pass result (with no `-ffp-contract` flag): worst fbm ULP = 3584,
seven failures. The diagnostic pattern — drift large near zero
crossings, small (10..100 ULP) on typical samples, hash bucket
unaffected — pointed at fused multiply-add. Apple Silicon Clang fuses
`a*b + c` into a single `fmadd` instruction by default (the chip has a
single-rounded FMA unit); emcc's wasm32 backend does not (FMA is an
optional WebAssembly extension, not enabled in the baseline target);
the firmware target (Cortex-M7) has no FPU FMA at all.

`raw_eval()` is dense with this pattern: the rotation update, the
gradient chain rule, the erosion divisor, the per-octave amplitude
multiply. One contraction on the host but not on wasm32 changed the
rounding of every downstream operation. The effect was bit-visible
even though the absolute drift was sub-mm (visually fine for rendering;
broken for the byte-equal determinism contract).

Adding `-ffp-contract=off` to the library's compile options forces
Clang to emit separate `fmul` + `fadd` everywhere, matching wasm32's
baseline semantics AND the firmware build. The reference vector was
regenerated; native and wasm32 are now bit-identical for all 100
fixtures, and the firmware (which never had FMA in the first place)
follows the same arithmetic path.

This is stronger than what the Shared Source Contract demands. The
contract permits ~1 ULP of drift on the fBm bucket; we achieve zero.
The 64-ULP tolerance budget in the harness stays as a safety margin
against future toolchain changes that might re-introduce small drift
— a regression that crosses 64 ULP is worth investigating; one that
stays under it can be accepted by re-running `--generate`.

### How to re-verify

```bash
# Native:
cmake --build src/terrain/build-native
./src/terrain/build-native/determinism_test                 # PASS

# WASM (headless, Node):
emcmake cmake -S src/terrain -B src/terrain/build-wasm
cmake --build src/terrain/build-wasm
node tests/terrain/determinism_test_node.mjs                # PASS

# WASM (browser, manual):
(cd src/terrain/build-wasm && python3 -m http.server 8000)
# open http://localhost:8000/determinism_test.html — verdict at top of page
```

## Per-call performance

Measured with `tests/terrain/perf_wasm_node.mjs` on a Release-build
wasm32 module loaded into Node 25.8.1 (V8) on `aarch64-apple-darwin`.
Default SIH parameters (`amp=20, wavelength=200, oct=6, hurst=0.7,
erosion=1, seed=42, home=(0,0)`):

| Mode | Iterations | Total | Per-call | Notes |
|---|---|---|---|---|
| pure WASM (`terrain_benchmark(N)`) | 1,000,000 | 365 ms | **0.365 µs** | No JS<->WASM marshalling — the tight C-side loop is what a heightmap-prebake path pays |
| JS-crossed (`Module._terrain(n, e)` in a JS loop) | 200,000 | 70 ms | **0.351 µs** | Includes the full JS<->WASM boundary on every call — what a per-vertex JS evaluation path pays |

The spec budget was **< 50 µs per call in the browser JS
context**. We measure **0.35 µs**, ~140× under budget. The JS<->WASM
boundary cost is effectively free for this call shape (two scalar
arguments, one scalar return, no heap traffic) — every microsecond is
spent in the noise math itself.

V8 in Node is a reasonable proxy for in-browser V8 (Chrome/Edge). Safari
(JavaScriptCore) and Firefox (SpiderMonkey) may differ by ~2× either
way; the rendering engineer should re-measure in the actual target
browser once the WASM module ships, but the order of magnitude here is
representative.

### Per-frame budget implications

At 60 FPS the renderer has 16.7 ms per frame. JS-side `terrain()` evaluation
spends that budget as follows:

| Vertices evaluated / frame | Pure WASM | JS-crossed | % of 16.7 ms frame |
|---|---|---|---|
| 256 | 0.09 ms | 0.09 ms | 0.5% |
| 1,024 | 0.37 ms | 0.36 ms | 2.2% |
| 4,096 | 1.49 ms | 1.44 ms | 8.9% |
| 16,384 | 5.98 ms | 5.74 ms | 36% |
| 65,025 (one 255×255 clipmap ring) | 23.7 ms | 22.8 ms | **143% — over budget** |
| 130,050 (two clipmap rings) | 47.5 ms | 45.6 ms | **284%** |

### Guidance for the renderer

- **Up to ~4,000 vertices/frame**: per-vertex evaluation from JS (either
  with a CPU heightmap baked into a VBO or with `Module._terrain` called
  inline) is fine. Stays under ~10% of the frame budget.
- **4,000 — 32,000 vertices/frame**: still feasible but uses a noticeable
  share of the frame. Worth profiling the rest of the draw cost first.
- **More than ~32,000 vertices/frame** (which includes a full 255×255
  clipmap ring, the spec's baseline): per-frame CPU evaluation is
  impractical. Bake heights to a `R32F` heightmap texture once per
  clipmap-translation event (camera moved past a vertex-spacing
  threshold) and sample in the vertex shader. The bake itself can be
  amortized across many frames since the clipmap doesn't move every
  frame.
- For the **3-ring clipmap (~195k vertices total)**: the CPU bake is the
  only viable path. A single bake costs ~70 ms (still cheap because it
  runs once per displacement event, not per frame). The texture upload
  fits in a `R32F` 256×256 (256 KB per ring × 3 = 0.75 MB) — well under
  the 2 MB billboard budget.
- The Lipschitz-bound max-slope output from `terrain_set_params()` lets
  the renderer compute its own "how far can the camera move before the
  bake is stale?" check without a per-frame full re-evaluation.

### How to re-run the perf harness

```bash
emcmake cmake -S src/terrain -B src/terrain/build-wasm -DCMAKE_BUILD_TYPE=Release
cmake --build src/terrain/build-wasm
node tests/terrain/perf_wasm_node.mjs
```
