// Headless perf harness for the wasm32 terrain build.
//
// Measures two costs:
//   (a) PURE WASM: terrain_benchmark(N) loops inside WASM, so each call
//       pays only the noise math — no JS<->WASM marshalling overhead.
//       This is the cost the renderer pays if heights are baked into a
//       texture via a tight C-side loop and then sampled in shader.
//   (b) JS-CROSSED: Module._terrain(n, e) called N times in a JS for-loop,
//       so each call pays the JS<->WASM boundary cost plus the math.
//       This is the cost the renderer pays per vertex IF heights are
//       evaluated one at a time from JS (the worst case for the
//       per-vertex shader callback strategy).
//
// Reports avg per-call latency in microseconds for both, plus the
// implied per-frame budget at typical clipmap vertex counts.
//
// Usage:
//   source $EMSDK_ROOT/emsdk_env.sh
//   emcmake cmake -S src/terrain -B src/terrain/build-wasm
//   cmake --build src/terrain/build-wasm -j
//   node tests/terrain/perf_wasm_node.mjs

import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";
import { performance } from "node:perf_hooks";

const __dirname = dirname(fileURLToPath(import.meta.url));
const WASM_DIR  = join(__dirname, "..", "..", "src", "terrain", "build-wasm");

const moduleUrl = new URL("file://" + join(WASM_DIR, "determinism_test.js"));
const { default: createModule } = await import(moduleUrl);
const Module = await createModule();

// Default SIH-realistic params: matches the firmware MVP default config.
Module._terrain_set_params(20.0, 200.0, 6, 0.7, 1.0, 42, 0.0, 0.0);

function timeBlock(label, fn) {
  // Warm-up to let the JIT settle.
  fn();
  const t0 = performance.now();
  fn();
  const t1 = performance.now();
  return { label, ms: t1 - t0 };
}

const PURE_ITERS  = 1_000_000;
const CROSS_ITERS = 200_000;

const pure  = timeBlock("pure-wasm", () => Module._terrain_benchmark(PURE_ITERS));
const cross = timeBlock("js-crossed", () => {
  let acc = 0;
  for (let i = 0; i < CROSS_ITERS; ++i) {
    const n = ((i & 0x3fff) >>> 0) * 0.123;
    const e = (((i * 17) >>> 0) & 0x3fff) * 0.071;
    acc += Module._terrain(n, e);
  }
  // prevent JIT from dropping the loop as dead
  if (Number.isNaN(acc)) process.exit(2);
});

const usPerCallPure  = (pure.ms  * 1000) / PURE_ITERS;
const usPerCallCross = (cross.ms * 1000) / CROSS_ITERS;

console.log("[perf] terrain() default SIH params (amp=20, wl=200, oct=6, hurst=0.7, erosion=1)");
console.log(`[perf] pure WASM  : ${PURE_ITERS}  calls in ${pure.ms.toFixed(2)} ms`);
console.log(`[perf]              -> ${usPerCallPure.toFixed(3)} us / call (no JS marshalling)`);
console.log(`[perf] JS-crossed : ${CROSS_ITERS}  calls in ${cross.ms.toFixed(2)} ms`);
console.log(`[perf]              -> ${usPerCallCross.toFixed(3)} us / call (JS<->WASM boundary)`);

console.log("");
console.log("[perf] implied per-frame budget at 60 FPS (16.7 ms):");
for (const v of [256, 1024, 4096, 65025, 130050]) {
  const mp = (v * usPerCallPure)  / 1000;   // ms
  const mc = (v * usPerCallCross) / 1000;
  console.log(`[perf]   ${String(v).padStart(7)} verts  ->  pure ${mp.toFixed(2)} ms   js-cross ${mc.toFixed(2)} ms`);
}
console.log("");
console.log("[perf] reference: budget is <50 us per browser-side call.");
