#!/usr/bin/env python3
"""
Terrain integration smoke.

Checks the parts of the replay → render pipeline that are verifiable without
a GUI or display server. Confirming that terrain is visually on screen needs
a real window and is documented separately in `baselines/README.md`.

What this validates:

  1. The fixture exposes the expected SIH_TERR_* values:
     - Initial (boot) snapshot: defaults (EN=0, AMP=0, ...).
     - changed_parameters at t≈7.1 s: EN=1, AMP=20, SEED=3, MPC_ALT_MODE=1.

  2. The ULog and MAVLink unit tests pass
     (`ulog_extract` + `mavlink_listener` in the C ctest harness).

  3. The hawkeye binary on disk has the orchestration symbols linked in
     (`hawkeye_terrain_set_apply_callback`, `terrain_renderer_apply_params`,
     `terrain_renderer_set_drone_velocity_ned`, `terrain_renderer_init/
     draw/shutdown`, `terrain_set_params`).

  4. The end-to-end NATIVE ingestion → renderer chain is in place:
     - main.c registers `terrain_renderer_apply_params` as the apply callback
       on the shared library once at startup.
     - data_source_ulog.c calls `hawkeye_terrain_apply_params()` on log load.
     - mavlink_receiver.c routes PARAM_VALUE into `terrain_params_on_param_value()`
       (the listener applies internally on snapshot complete / value change).

  5. The end-to-end WASM ingestion → renderer chain is in place:
     - wasm/wasm_main.c registers the same callback as native, mirroring the
       src/main.c registration after the canvas size set + before scene_init.
     - wasm/wasm_main.c uses the byte-buffer extractor variant
       (`ulog_extract_terrain_params_from_buffer`) since WASM loads `.ulg`
       from a JS Uint8Array, not a file path.
     - wasm/wasm_main.c calls `hawkeye_terrain_apply_params()` on load, which
       fires the registered callback and updates the renderer's local cache.

Run:
  python3 tests/terrain/integration_smoke.py

Exits 0 on PASS, 1 on any FAIL.
"""

from __future__ import annotations

import math
import os
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
FIXTURE = HERE / "fixtures" / "sih_terrain_on.ulg"
HAWKEYE = REPO / "build" / "hawkeye"
MAIN_C = REPO / "src" / "main.c"
DATA_SOURCE_ULOG_C = REPO / "src" / "data_source_ulog.c"
MAVLINK_RECEIVER_C = REPO / "src" / "mavlink_receiver.c"
WASM_MAIN_C = REPO / "wasm" / "wasm_main.c"

# changed_parameters wire-encode ints as float bit patterns (PARAM_SET wire
# convention). 1065353216 == IEEE-754 bits of float 1.0; 1077936128 == 3.0.
BITS_FLOAT_1 = 1065353216
BITS_FLOAT_3 = 1077936128

EXPECTED_INITIAL = {
    "SIH_TERR_EN":      0,
    "SIH_TERR_AMP":     0.0,
    "SIH_TERR_FREQ":    0.005,
    "SIH_TERR_OCT":     6,
    "SIH_TERR_HURST":   0.7,
    "SIH_TERR_EROSION": 1.0,
    "SIH_TERR_SEED":    0,
    "MPC_ALT_MODE":     2,
}

EXPECTED_CHANGED_AT_T7 = {
    "SIH_TERR_EN":     BITS_FLOAT_1,
    "SIH_TERR_AMP":    20.0,
    "SIH_TERR_SEED":   BITS_FLOAT_3,
    "MPC_ALT_MODE":    BITS_FLOAT_1,
}

EXPECTED_BINARY_SYMBOLS = [
    "hawkeye_terrain_set_apply_callback",
    "terrain_renderer_apply_params",
    "terrain_renderer_set_drone_velocity_ned",
    "terrain_renderer_init",
    "terrain_renderer_draw",
    "terrain_renderer_shutdown",
    "terrain_set_params",
]

EXPECTED_CHAIN_NATIVE = [
    # main.c: callback registration after window init. This is the only direct
    # renderer-apply call site after the double-fire cleanup refactor — both
    # ingestion paths fan out automatically via the shared library callback.
    (MAIN_C,
     r"hawkeye_terrain_set_apply_callback\s*\(\s*terrain_renderer_apply_params\s*\)"),
    # data_source_ulog.c: ULog ingestion calls into the shared library on load.
    (DATA_SOURCE_ULOG_C,
     r"hawkeye_terrain_apply_params\s*\(\s*&\s*terr\s*\)"),
    # mavlink_receiver.c: PARAM_VALUE messages route to the listener, which
    # applies internally on initial-snapshot-complete or any value change.
    (MAVLINK_RECEIVER_C,
     r"terrain_params_on_param_value\s*\("),
]

EXPECTED_CHAIN_WASM = [
    # wasm/wasm_main.c registers the same callback as native, mirroring
    # src/main.c after the canvas size set + before scene_init.
    (WASM_MAIN_C,
     r"hawkeye_terrain_set_apply_callback\s*\(\s*terrain_renderer_apply_params\s*\)"),
    # wasm/wasm_main.c uses the byte-buffer extractor variant (WASM loads
    # `.ulg` from a JS Uint8Array, not a file path).
    (WASM_MAIN_C,
     r"ulog_extract_terrain_params_from_buffer\s*\("),
    # wasm/wasm_main.c calls the shared library apply on log load, which
    # fires the registered callback and updates the renderer's local cache.
    (WASM_MAIN_C,
     r"hawkeye_terrain_apply_params\s*\(\s*&\s*terr\s*\)"),
]


# ----- check helpers -----------------------------------------------------

class Failed(Exception):
    pass

def _ok(msg):
    print(f"  PASS  {msg}")

def _fail(msg):
    print(f"  FAIL  {msg}")
    raise Failed(msg)

def _approx(a, b, tol=1e-3):
    return abs(float(a) - float(b)) <= tol


# ----- checks ------------------------------------------------------------

def check_fixture_params():
    print("[1] fixture SIH_TERR_* schema (pyulog)")
    try:
        from pyulog import ULog
    except ImportError:
        _fail("pyulog not installed; install with: pip install pyulog")
    if not FIXTURE.exists():
        _fail(f"fixture missing at {FIXTURE}")
    u = ULog(str(FIXTURE))

    for name, expected in EXPECTED_INITIAL.items():
        got = u.initial_parameters.get(name)
        if got is None:
            _fail(f"initial_parameters[{name!r}] missing")
        if isinstance(expected, float):
            if not _approx(got, expected, tol=0.01):
                _fail(f"initial_parameters[{name!r}] = {got!r}, expected ~{expected}")
        else:
            if int(got) != int(expected):
                _fail(f"initial_parameters[{name!r}] = {got!r}, expected {expected}")
    _ok("initial_parameters match defaults")

    seen = {}
    for (t_us, name, val) in getattr(u, "changed_parameters", []):
        if name in EXPECTED_CHANGED_AT_T7:
            seen[name] = (t_us / 1e6, val)
    for name, expected in EXPECTED_CHANGED_AT_T7.items():
        if name not in seen:
            _fail(f"changed_parameters[{name!r}] not present")
        t_s, val = seen[name]
        if not (6.0 < t_s < 8.0):
            _fail(f"changed_parameters[{name!r}] timestamp {t_s:.2f}s out of [6.0, 8.0]")
        if isinstance(expected, float):
            if not _approx(val, expected, tol=0.01):
                _fail(f"changed_parameters[{name!r}] = {val!r}, expected ~{expected}")
        else:
            if int(val) != int(expected):
                _fail(f"changed_parameters[{name!r}] = {val!r}, expected {expected}")
    _ok(f"changed_parameters land at t≈7s with renderer-enable values "
        f"(EN={BITS_FLOAT_1}/=float 1.0, SEED={BITS_FLOAT_3}/=float 3.0)")


def check_ctest():
    print("[2] ULog + MAVLink unit tests (ctest)")
    build = REPO / "build"
    if not build.exists():
        _fail("build/ missing — run `cmake -B build -DBUILD_TESTING=ON && cmake --build build` first")
    proc = subprocess.run(
        ["ctest", "--test-dir", str(build), "-C", "Release",
         "-R", "ulog_extract|mavlink_listener", "--output-on-failure"],
        capture_output=True, text=True,
    )
    if proc.returncode != 0:
        sys.stdout.write(proc.stdout)
        sys.stdout.write(proc.stderr)
        _fail("ctest -R 'ulog_extract|mavlink_listener' failed")
    if "100% tests passed" not in proc.stdout:
        _fail("ctest did not report 100% — see output above")
    _ok("ulog_extract + mavlink_listener PASS")


def check_binary_symbols():
    print("[3] orchestration symbols linked into hawkeye binary")
    if not HAWKEYE.exists():
        _fail(f"binary missing at {HAWKEYE} — `cmake --build build` first")
    proc = subprocess.run(["nm", str(HAWKEYE)], capture_output=True, text=True)
    if proc.returncode != 0:
        _fail(f"nm failed: {proc.stderr}")
    missing = []
    for sym in EXPECTED_BINARY_SYMBOLS:
        # nm on macOS prefixes underscores; tolerate both.
        if re.search(rf"\b_?{re.escape(sym)}\b", proc.stdout) is None:
            missing.append(sym)
    if missing:
        _fail(f"missing symbols: {missing}")
    _ok(f"all {len(EXPECTED_BINARY_SYMBOLS)} symbols present")


def _check_chain(label, chain):
    missing = []
    for path, pat in chain:
        if not path.exists():
            _fail(f"source file missing at {path}")
        if re.search(pat, path.read_text()) is None:
            missing.append((path.relative_to(REPO), pat))
    if missing:
        for p, pat in missing:
            print(f"        - {p}: {pat}")
        _fail(f"{label}: missing {len(missing)} chain link(s)")


def check_orchestration_chain_native():
    print("[4] ingestion → renderer chain (native build)")
    _check_chain("native", EXPECTED_CHAIN_NATIVE)
    _ok("callback registration + ULog apply + MAVLink listener route all present")


def check_orchestration_chain_wasm():
    print("[5] ingestion → renderer chain (WASM build)")
    _check_chain("wasm", EXPECTED_CHAIN_WASM)
    _ok("callback registration + buffer-extract + apply all present in wasm_main.c")


# ----- main --------------------------------------------------------------

def main():
    print("Terrain integration smoke")
    print(f"  repo      = {REPO}")
    print(f"  fixture   = {FIXTURE.relative_to(REPO)}")
    print(f"  binary    = {HAWKEYE.relative_to(REPO)}")
    print()
    try:
        check_fixture_params()
        check_ctest()
        check_binary_symbols()
        check_orchestration_chain_native()
        check_orchestration_chain_wasm()
    except Failed:
        print()
        print("RESULT: FAIL")
        return 1
    print()
    print("RESULT: PASS — runtime path verified for SIH-log replay")
    print("        (visual baseline capture deferred — see baselines/README.md)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
