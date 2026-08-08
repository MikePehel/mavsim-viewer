// Headless smoke harness for the wasm32 build of the terrain determinism test.
//
// Mirrors the browser harness (tests/terrain/determinism_test.html): loads
// the emscripten module, runs the 100 fixtures, compares to the committed
// native-generated reference bit patterns, prints the verdict + worst ULP.
// Exits 0 on PASS, 1 on FAIL.
//
// Parsing / ULP math / comparison are shared with the browser harness via
// determinism_compare.mjs — this file only does the Node-specific bits
// (read the reference via fs, load the wasm module, print to stdout).
//
// Usage (from repo root, after building the WASM target):
//   source $EMSDK_ROOT/emsdk_env.sh
//   emcmake cmake -S src/terrain -B src/terrain/build-wasm
//   cmake --build src/terrain/build-wasm -j
//   node tests/terrain/determinism_test_node.mjs

import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";
import {
  parseReferenceText, compareBits, bitsToFloat, hex8, MAX_ULP_TOLERANCE,
} from "./determinism_compare.mjs";

const __dirname = dirname(fileURLToPath(import.meta.url));
const WASM_DIR  = join(__dirname, "..", "..", "src", "terrain", "build-wasm");

const REFERENCE_BITS = parseReferenceText(
  readFileSync(join(__dirname, "determinism_reference.h"), "utf8"));
if (REFERENCE_BITS.length !== 100) {
  console.error(`unexpected reference array length ${REFERENCE_BITS.length} (want 100)`);
  process.exit(1);
}

const moduleUrl = new URL("file://" + join(WASM_DIR, "determinism_test.js"));
const { default: createModule } = await import(moduleUrl);
const Module = await createModule();

const N            = Module._determinism_num_inputs();
const numHashExact = Module._determinism_num_hash_exact();
if (N !== REFERENCE_BITS.length) {
  console.error(`fixture count mismatch: wasm=${N} reference=${REFERENCE_BITS.length}`);
  process.exit(1);
}

const ptr = Module._malloc(N * 4);
Module._determinism_run_into_buffer(ptr);
const actual = new Uint32Array(Module.HEAPU8.buffer, ptr, N).slice();
Module._free(ptr);

const r = compareBits(actual, REFERENCE_BITS, numHashExact);

for (const f of r.hashFailures) {
  console.log(`FAIL hash[${f.i}] expected ${hex8(f.expected)} got ${hex8(f.actual)}` +
              `  (${bitsToFloat(f.expected)} vs ${bitsToFloat(f.actual)})`);
}
for (const f of r.fbmFailures) {
  console.log(`FAIL fbm[${f.i}] ULP=${f.ulp}` +
              ` expected ${hex8(f.expected)} got ${hex8(f.actual)}`);
}

console.log("");
console.log(`[wasm-node] hash-exact: ${r.hashPass}/${numHashExact} PASS`);
console.log(`[wasm-node] fbm-drift:  ${r.fbmPass}/${N - numHashExact} PASS` +
            `  (worst ULP = ${r.worstUlp} at idx ${r.worstIdx}; tolerance ${MAX_ULP_TOLERANCE})`);

if (r.failHash !== 0) {
  console.log(`[wasm-node] FAIL: hash-exact regression — integer hash bit pattern changed.`);
}
if (r.failFbm !== 0) {
  console.log(`[wasm-node] FAIL: fBm ULP drift exceeds tolerance.`);
}

process.exit((r.failHash + r.failFbm) === 0 ? 0 : 1);
