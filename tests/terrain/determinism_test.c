/*
 * Determinism harness for the vendored terrain library.
 *
 * Why this exists
 * ---------------
 * The shared `terrain.c` is the contract between the SIH firmware and the
 * viewer. If the viewer build produces a different surface than the firmware
 * build, the "replay shows the surface the drone flew over" promise breaks.
 * This harness verifies the contract on both build targets the viewer
 * compiles into (native host and wasm32) against a reference set of 100
 * fixed inputs.
 *
 * Two buckets of inputs
 * ---------------------
 *   - HASH-EXACT  (50 cases): 1-octave noise sampled at integer (N, E) with
 *     `wavelength = 1`, `erosion = 0`. In these conditions value_noise_d
 *     reduces to a single `hash_to_float(hash2d(ix, iy, seed))` lookup, and
 *     the integer hash is bit-exact across compilers / architectures by
 *     construction. We REQUIRE byte-equality of the IEEE-754 bit pattern
 *     here; any drift is a portability regression in the shared source.
 *
 *   - FBM-DRIFT   (50 cases): multi-octave fBm at fractional (N, E) with
 *     typical SIH parameter ranges. Float accumulation order and FPU
 *     rounding can produce 1..N ULP of drift between architectures, so
 *     we allow up to `MAX_ULP_TOLERANCE` ULP and report the worst
 *     observed.
 *
 * The reference values
 * --------------------
 * `determinism_reference.h` holds the 100 expected IEEE-754 bit patterns
 * (as uint32_t — exact equality regardless of FP env). It was generated
 * once on x86_64 macOS native by running this binary with `--generate`,
 * and is committed alongside the test source. Re-running with `--generate`
 * after any intentional change to the math regenerates it.
 *
 * Modes
 * -----
 *   ./determinism_test               -> verify; exits 0 on PASS, 1 on FAIL
 *   ./determinism_test --generate    -> print fresh reference header to stdout
 *
 * WASM mode (DETERMINISM_TEST_WASM defined):
 *   Exports `determinism_run_into_buffer(uint32_t *out)` which fills `out`
 *   with NUM_INPUTS uint32_t bit patterns. The HTML harness calls this and
 *   compares to the reference embedded in the page.
 */

#include "terrain.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * Inputs.
 *
 * Constructed deterministically from an index 0..NUM_INPUTS-1 so the test
 * fixture is reproducible and inspectable without serializing 100 hand-
 * written records. The two buckets straddle the array (first 50 are
 * hash-exact; last 50 are fbm-drift).
 * --------------------------------------------------------------------------- */

#define NUM_HASH_EXACT 50
#define NUM_FBM_DRIFT  50
#define NUM_INPUTS     (NUM_HASH_EXACT + NUM_FBM_DRIFT)

/* ULP budget for fBm cases. ARM-vs-x86 typically drifts 1-4 ULP per
 * accumulated octave; 64 absorbs that even at the high-octave end while
 * still catching any systemic divergence (a leaked -ffast-math would
 * push drift into the thousands). */
#define MAX_ULP_TOLERANCE 64u

typedef struct {
	float amp;
	float wavelength;
	int   oct;
	float hurst;
	float erosion;
	int   seed;
	float home_n;
	float home_e;
	float n;
	float e;
	int   hash_exact;     /* 1 = bit-exact required; 0 = ULP-tolerant */
} terrain_input_t;

static void make_input(int idx, terrain_input_t *out)
{
	if (idx < NUM_HASH_EXACT) {
		/* Hash-exact bucket: sample value_noise_d at integer (N, E)
		 * with wavelength=1, oct=1, erosion=0. raw_eval reduces to a
		 * single value_noise_d call whose output simplifies to
		 *   k0 = hash_to_float(hash2d(ix, iy, seed))
		 * because xf == yf == 0 forces u == v == 0. Bit-exact across
		 * any IEEE-754 platform that respects 32-bit unsigned wraparound. */
		out->amp        = 1.f;
		out->wavelength = 1.f;
		out->oct        = 1;
		out->hurst      = 0.7f;
		out->erosion    = 0.f;
		out->seed       = 1000 + idx;
		out->home_n     = 0.f;
		out->home_e     = 0.f;
		out->n          = (float)(((idx *  7) % 53) - 26);
		out->e          = (float)(((idx * 11) % 41) - 20);
		out->hash_exact = 1;
	} else {
		int j = idx - NUM_HASH_EXACT;
		/* Realistic SIH parameter ranges. Walks amp / wavelength / oct /
		 * hurst / erosion / seed / home over the index space so the 50
		 * cases cover the configuration surface (not just one default). */
		out->amp        = 5.f + (float)(j % 4) * 5.f;          /* 5, 10, 15, 20 */
		out->wavelength = 50.f + (float)(j % 5) * 50.f;        /* 50..250 */
		out->oct        = 3 + (j % 5);                         /* 3..7 */
		out->hurst      = 0.5f + 0.1f * (float)(j % 6);        /* 0.5..1.0 */
		out->erosion    = 0.5f * (float)((j % 3) + 1);         /* 0.5, 1.0, 1.5 */
		out->seed       = 2000 + j;
		out->home_n     = (float)(((j * 13) % 71) - 35);
		out->home_e     = (float)(((j * 17) % 61) - 30);
		out->n          = (float)(((j * 37) % 197) - 98) + 0.5f;
		out->e          = (float)(((j * 41) % 173) - 86) + 0.5f;
		out->hash_exact = 0;
	}
}

/* ---------------------------------------------------------------------------
 * Run all inputs and produce the result vector as IEEE-754 bit patterns.
 *
 * Bit-pattern (not float) is the unit so verification is exact comparison
 * regardless of compiler-injected float promotion / x87 80-bit shenanigans.
 * --------------------------------------------------------------------------- */

static uint32_t float_bits(float f)
{
	uint32_t u;
	memcpy(&u, &f, sizeof(u));
	return u;
}

static float bits_to_float(uint32_t u)
{
	float f;
	memcpy(&f, &u, sizeof(f));
	return f;
}

static void run_one(const terrain_input_t *in, uint32_t *out_bits)
{
	terrain_set_params(in->amp, in->wavelength, in->oct,
			   in->hurst, in->erosion, in->seed,
			   in->home_n, in->home_e);
	float h = terrain(in->n, in->e);
	*out_bits = float_bits(h);
}

static void run_all(uint32_t *out_bits /* NUM_INPUTS slots */)
{
	terrain_input_t in;

	for (int i = 0; i < NUM_INPUTS; ++i) {
		make_input(i, &in);
		run_one(&in, &out_bits[i]);
	}
}

/* ---------------------------------------------------------------------------
 * WASM exports.
 * --------------------------------------------------------------------------- */

#ifdef DETERMINISM_TEST_WASM
#  ifdef __EMSCRIPTEN__
#    include <emscripten/emscripten.h>
#    define EXPORT EMSCRIPTEN_KEEPALIVE
#  else
#    define EXPORT
#  endif

EXPORT void determinism_run_into_buffer(uint32_t *out)
{
	run_all(out);
}

EXPORT unsigned int determinism_num_inputs(void)
{
	return (unsigned int)NUM_INPUTS;
}

EXPORT unsigned int determinism_num_hash_exact(void)
{
	return (unsigned int)NUM_HASH_EXACT;
}

/* Tight in-WASM loop for measuring pure terrain() cost without paying
 * the JS<->WASM marshalling overhead on every call. The (n, e) inputs
 * drift across iterations so the optimizer can't const-fold the whole
 * loop into one call. Returns the running accumulator so the optimizer
 * also can't drop the calls as dead code. */
EXPORT float terrain_benchmark(unsigned int n_iterations)
{
	float acc = 0.f;

	for (unsigned int i = 0; i < n_iterations; ++i) {
		float fn = (float)(int)(i & 0x3fffu) * 0.123f;
		float fe = (float)(int)((i * 17u) & 0x3fffu) * 0.071f;
		acc += terrain(fn, fe);
	}

	return acc;
}
#endif

/* ---------------------------------------------------------------------------
 * ULP distance for the fBm tolerance check.
 *
 * IEEE-754 monotonic-bit-pattern trick: nudging a positive float by one
 * step changes its bit pattern by one. Sign-aware so negatives compare
 * sanely.
 * --------------------------------------------------------------------------- */

static uint32_t ulp_distance(uint32_t a, uint32_t b)
{
	/* Convert sign-magnitude to two's-complement-like ordering so |a - b|
	 * is the number of representable floats between them. */
	int32_t ai = (a & 0x80000000u) ? (int32_t)(0x80000000u - (a & 0x7fffffffu))
				       : (int32_t)a;
	int32_t bi = (b & 0x80000000u) ? (int32_t)(0x80000000u - (b & 0x7fffffffu))
				       : (int32_t)b;
	int32_t d  = ai - bi;
	return (uint32_t)(d < 0 ? -d : d);
}

/* ---------------------------------------------------------------------------
 * Generate mode: print a fresh determinism_reference.h header to stdout.
 *
 * The format is a single static const array of uint32_t bit patterns.
 * Includes a banner with the host triple and date so it is obvious how
 * the reference was generated.
 * --------------------------------------------------------------------------- */

#ifndef DETERMINISM_TEST_WASM
static void emit_generate_header(void)
{
	uint32_t bits[NUM_INPUTS];
	run_all(bits);

	printf("/* AUTO-GENERATED by determinism_test --generate.\n");
	printf(" *\n");
	printf(" * %d reference IEEE-754 bit patterns for terrain(n, e), one\n", NUM_INPUTS);
	printf(" * per input in determinism_test.c's make_input() index space.\n");
	printf(" * Indices 0..%d are HASH-EXACT (bit-equality required across\n", NUM_HASH_EXACT - 1);
	printf(" * platforms). Indices %d..%d are FBM-DRIFT (ULP-tolerant).\n",
	       NUM_HASH_EXACT, NUM_INPUTS - 1);
	printf(" *\n");
	printf(" * To regenerate (after an intentional change to the shared math):\n");
	printf(" *   cmake -S src/terrain -B src/terrain/build-native\n");
	printf(" *   cmake --build src/terrain/build-native\n");
	printf(" *   ./src/terrain/build-native/determinism_test --generate \\\n");
	printf(" *       > tests/terrain/determinism_reference.h\n");
	printf(" */\n\n");
	printf("#ifndef HAWKEYE_TESTS_TERRAIN_DETERMINISM_REFERENCE_H_\n");
	printf("#define HAWKEYE_TESTS_TERRAIN_DETERMINISM_REFERENCE_H_\n\n");
	printf("#include <stdint.h>\n\n");
	printf("static const uint32_t DETERMINISM_REFERENCE_BITS[%d] = {\n", NUM_INPUTS);

	for (int i = 0; i < NUM_INPUTS; ++i) {
		const char *tag = (i < NUM_HASH_EXACT) ? "hash" : "fbm ";
		printf("\t0x%08xu,  /* [%3d] %s  -> %.9g */\n",
		       bits[i], i, tag, (double)bits_to_float(bits[i]));
	}

	printf("};\n\n");
	printf("#endif /* HAWKEYE_TESTS_TERRAIN_DETERMINISM_REFERENCE_H_ */\n");
}
#endif

/* ---------------------------------------------------------------------------
 * Verify mode: compare run_all() output to the committed reference array.
 * --------------------------------------------------------------------------- */

#include "determinism_reference.h"

#ifndef DETERMINISM_TEST_WASM
static int run_verify(void)
{
	uint32_t actual[NUM_INPUTS];
	run_all(actual);

	int      fail_hash = 0;
	int      fail_fbm  = 0;
	uint32_t worst_ulp = 0;
	int      worst_idx = -1;

	for (int i = 0; i < NUM_INPUTS; ++i) {
		terrain_input_t in;
		make_input(i, &in);

		uint32_t a = actual[i];
		uint32_t e = DETERMINISM_REFERENCE_BITS[i];

		if (in.hash_exact) {
			if (a != e) {
				++fail_hash;
				printf("FAIL hash[%3d] expected 0x%08x got 0x%08x  (%.9g vs %.9g)\n",
				       i, e, a,
				       (double)bits_to_float(e),
				       (double)bits_to_float(a));
			}
		} else {
			uint32_t d = ulp_distance(a, e);

			if (d > worst_ulp) {
				worst_ulp = d;
				worst_idx = i;
			}

			if (d > MAX_ULP_TOLERANCE) {
				++fail_fbm;
				printf("FAIL fbm[%3d]  expected 0x%08x got 0x%08x  ULP=%u (%.9g vs %.9g)\n",
				       i, e, a, d,
				       (double)bits_to_float(e),
				       (double)bits_to_float(a));
			}
		}
	}

	printf("\n[determinism_test] hash-exact: %d/%d PASS\n",
	       NUM_HASH_EXACT - fail_hash, NUM_HASH_EXACT);
	printf("[determinism_test] fbm-drift:  %d/%d PASS  (worst ULP = %u at idx %d; tolerance %u)\n",
	       NUM_FBM_DRIFT - fail_fbm, NUM_FBM_DRIFT,
	       worst_ulp, worst_idx, MAX_ULP_TOLERANCE);

	if (fail_hash != 0) {
		printf("[determinism_test] FAIL: hash-exact regression — integer hash bit pattern changed.\n");
		printf("                       investigate the shared source for unsigned-arithmetic or\n");
		printf("                       compiler-flag drift before touching the reference vector.\n");
	}

	if (fail_fbm != 0) {
		printf("[determinism_test] FAIL: fBm ULP drift exceeds tolerance.\n");
		printf("                       a leaked -ffast-math or x87 80-bit promotion is the usual cause.\n");
	}

	return (fail_hash + fail_fbm) == 0 ? 0 : 1;
}
#endif

/* ---------------------------------------------------------------------------
 * Entry point.
 * --------------------------------------------------------------------------- */

int main(int argc, char **argv)
{
#ifndef DETERMINISM_TEST_WASM
	if (argc >= 2 && strcmp(argv[1], "--generate") == 0) {
		emit_generate_header();
		return 0;
	}

	return run_verify();
#else
	(void)argc;
	(void)argv;
	/* In WASM mode main() is a no-op; the page calls
	 * determinism_run_into_buffer() directly via cwrap. */
	return 0;
#endif
}
