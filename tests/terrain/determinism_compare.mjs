// Shared compare logic for the terrain determinism harnesses.
//
// Both the Node smoke (determinism_test_node.mjs) and the browser page
// (determinism_test.html) load the same wasm32 build of the vendored
// terrain.c, run the 100 fixtures, and compare the output bit patterns to
// the committed native-generated reference. The *only* thing that differs
// between the two runtimes is how they obtain the reference text (node:fs
// vs fetch) and how they surface the verdict (stdout vs DOM). Everything
// below — parsing, ULP math, and the pass/fail comparison — is identical,
// so it lives here once instead of being copy-pasted into each harness.

export const MAX_ULP_TOLERANCE = 64;

// Parse a determinism_reference.h into a Uint32Array. Matches the `0x........u`
// literals the C generator (determinism_test --generate) emits. This keeps the
// reference single-sourced: there is exactly one copy of the expected bits,
// the .h file, read by C (#include), Node (fs), and the browser (fetch).
export function parseReferenceText(text) {
  const re   = /0x([0-9a-fA-F]{8})u/g;
  const bits = [];
  let m;
  while ((m = re.exec(text)) !== null) {
    bits.push(parseInt(m[1], 16) >>> 0);
  }
  return new Uint32Array(bits);
}

// Sign-aware bit-pattern distance, matching the native test's ulp_distance().
// BigInt subtraction dodges JS 32-bit overflow at the extremes.
export function ulpDistance(a, b) {
  const ai = (a & 0x80000000) ? (0x80000000 - (a & 0x7fffffff)) : a;
  const bi = (b & 0x80000000) ? (0x80000000 - (b & 0x7fffffff)) : b;
  const d  = BigInt(ai) - BigInt(bi);
  return Number(d < 0n ? -d : d);
}

export function bitsToFloat(u) {
  const buf = new ArrayBuffer(4);
  new DataView(buf).setUint32(0, u >>> 0, true);
  return new DataView(buf).getFloat32(0, true);
}

export function hex8(u) {
  return "0x" + (u >>> 0).toString(16).padStart(8, "0");
}

// Compare actual wasm output against the reference. Indices [0, numHashExact)
// must be bit-exact; the rest are ULP-tolerant (fBm drift). Returns a structured
// verdict; each runtime formats it however it likes. Because all hash-exact
// indices precede all fbm indices, hashFailures and fbmFailures are already in
// ascending-index order when concatenated.
export function compareBits(actual, reference, numHashExact, tolerance = MAX_ULP_TOLERANCE) {
  const N = reference.length;
  let failHash = 0, failFbm = 0, worstUlp = 0, worstIdx = -1;
  const hashFailures = [], fbmFailures = [], fbmUlps = [];

  for (let i = 0; i < N; ++i) {
    const a = actual[i] >>> 0;
    const e = reference[i] >>> 0;
    if (i < numHashExact) {
      if (a !== e) {
        ++failHash;
        hashFailures.push({ i, expected: e, actual: a });
      }
    } else {
      const d = ulpDistance(a, e);
      fbmUlps.push({ i, ulp: d, expected: e, actual: a });
      if (d > worstUlp) { worstUlp = d; worstIdx = i; }
      if (d > tolerance) {
        ++failFbm;
        fbmFailures.push({ i, ulp: d, expected: e, actual: a });
      }
    }
  }

  return {
    N, numHashExact,
    failHash, failFbm,
    hashPass: numHashExact - failHash,
    fbmPass:  (N - numHashExact) - failFbm,
    worstUlp, worstIdx,
    hashFailures, fbmFailures, fbmUlps,
    pass: failHash === 0 && failFbm === 0,
  };
}
