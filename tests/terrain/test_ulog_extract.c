/**
 * @file test_ulog_extract.c
 *
 * Unit tests for ulog_extract_terrain_params — ULog
 * parameter snapshot extractor.
 *
 * Covers:
 *   - the SIH-terrain fixture (terrain ON, runtime-changed values at
 *     t≈7.1 s) → struct populated with effective post-change values
 *   - a pre-terrain fixture (no SIH_TERR_EN) → returns false, struct
 *     left at defaults
 *   - a missing file → returns false, struct still at defaults
 *
 * The test provides its own stub for `terrain_set_params()` so it can
 * compile/link without the vendored terrain library (lands on
 * a separate branch). The stub captures the args from the most recent
 * call so the freq→wavelength inversion in apply_params can be verified.
 */

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "terrain/terrain_params.h"
#include "ulog/terrain_params_extract.h"

#define SIH_TERRAIN_LOG  TERRAIN_FIXTURES_DIR "/sih_terrain_on.ulg"
#define PRETERRAIN_LOG   FIXTURES_DIR        "/dde9a24c-34c5-4868-b09c-bd3481ed1029.ulg"

/* --- Stub for the vendored terrain library ------------------------------- */

static struct {
    int   calls;
    float amp;
    float wavelength;
    int   seed;
    float home_n;
    float home_e;
    float plane_deg;
} g_set_params;

void terrain_set_params(float amp, float wavelength, int seed,
                        float home_n, float home_e,
                        float plane_deg)
{
    g_set_params.calls++;
    g_set_params.amp        = amp;
    g_set_params.wavelength = wavelength;
    g_set_params.seed       = seed;
    g_set_params.home_n     = home_n;
    g_set_params.home_e     = home_e;
    g_set_params.plane_deg  = plane_deg;
}

static void reset_stub(void)
{
    memset(&g_set_params, 0, sizeof(g_set_params));
}

/* --- Helpers ------------------------------------------------------------- */

static bool approx_eq(float a, float b, float tol)
{
    return fabsf(a - b) <= tol;
}

/* --- Tests --------------------------------------------------------------- */

static void test_sih_terrain_on_log(void)
{
    HawkeyeTerrainParams p;
    bool ok = ulog_extract_terrain_params(SIH_TERRAIN_LOG, &p);
    assert(ok);

    /*
     * Expected values per tests/terrain/README.md (the fixture was
     * recorded with runtime PARAM_SETs at t≈7.1 s).
     * SIH_TERR_EN: 0 → 1 (tri-state: 1 = TERRAIN mode)
     * SIH_TERR_AMP: 0 → 20
     * SIH_TERR_SEED: 0 → 3
     * SIH_TERR_FREQ, OCT, HURST, EROSION unchanged from defaults.
     */
    assert(p.mode == HAWKEYE_TERRAIN_MODE_TERRAIN);
    assert(approx_eq(p.amp,     20.0f,  1e-3f));
    assert(approx_eq(p.freq,    0.005f, 1e-6f));
    /* oct/hurst/erosion no longer in the struct — those
     * are hard-coded noise internals in the vendored terrain.c.
     * restored amp/freq but left oct/hurst/erosion baked in. */
    assert(p.seed    == 3);
    /* home = (0, 0) because the viewer has no separate world origin yet */
    assert(p.home_n_m == 0.0f);
    assert(p.home_e_m == 0.0f);
    printf("  PASS sih_terrain_on_log (amp=%.1f seed=%d mode=%d)\n",
           p.amp, p.seed, p.mode);
}

static void test_apply_inverts_freq_to_wavelength(void)
{
    HawkeyeTerrainParams p;
    bool ok = ulog_extract_terrain_params(SIH_TERRAIN_LOG, &p);
    assert(ok);

    reset_stub();
    hawkeye_terrain_apply_params(&p);
    assert(g_set_params.calls == 1);

    /* freq = 0.005 → wavelength = 200 */
    assert(approx_eq(g_set_params.wavelength, 200.0f, 1e-3f));
    assert(approx_eq(g_set_params.amp,        20.0f,  1e-3f));
    assert(g_set_params.seed == 3);
    assert(g_set_params.home_n == 0.0f);
    assert(g_set_params.home_e == 0.0f);
    printf("  PASS apply_inverts_freq_to_wavelength (wavelength=%.1f m)\n",
           g_set_params.wavelength);
}

static void test_non_sih_log(void)
{
    HawkeyeTerrainParams p;
    bool ok = ulog_extract_terrain_params(PRETERRAIN_LOG, &p);
    assert(!ok);

    /* On failure the struct is left at safe defaults. */
    assert(p.mode == HAWKEYE_TERRAIN_MODE_OFF);
    assert(p.amp  == 0.0f);
    printf("  PASS non_sih_log (returns false, struct = defaults)\n");
}

static void test_missing_file(void)
{
    HawkeyeTerrainParams p;
    bool ok = ulog_extract_terrain_params("/nonexistent/path.ulg", &p);
    assert(!ok);
    assert(p.mode == HAWKEYE_TERRAIN_MODE_OFF);
    printf("  PASS missing_file (returns false)\n");
}

static void test_disabled_apply_forces_amp_zero(void)
{
    /*
     * Independent of any log: when mode != TERRAIN, apply_params must
     * push amp=0 to the underlying library regardless of the input amp.
     * This is the safety property the renderer relies on — fBm only runs
     * in TERRAIN mode (matches PX4 sih.cpp's terr_mode==1 gate).
     */
    HawkeyeTerrainParams p = hawkeye_terrain_params_default();
    p.mode    = HAWKEYE_TERRAIN_MODE_WALLS;  /* not TERRAIN — should force amp=0 */
    p.amp     = 50.0f;
    p.freq    = 0.01f;

    reset_stub();
    hawkeye_terrain_apply_params(&p);
    assert(g_set_params.calls == 1);
    assert(g_set_params.amp == 0.0f);
    /* wavelength still reflects the configured freq, not clamped */
    assert(approx_eq(g_set_params.wavelength, 100.0f, 1e-3f));
    printf("  PASS disabled_apply_forces_amp_zero\n");
}

static void test_apply_null_uses_defaults(void)
{
    reset_stub();
    hawkeye_terrain_apply_params(NULL);
    assert(g_set_params.calls == 1);
    assert(g_set_params.amp == 0.0f);  /* defaults => disabled => amp=0 */
    printf("  PASS apply_null_uses_defaults\n");
}

/* --- Apply-callback hook (renderer fan-out) ------------------------------ */

static struct {
    int   calls;
    int   mode;
    float amp;
    int   seed;
} g_cb_capture;

static void apply_callback(const HawkeyeTerrainParams *p)
{
    g_cb_capture.calls++;
    g_cb_capture.mode    = p->mode;
    g_cb_capture.amp     = p->amp;
    g_cb_capture.seed    = p->seed;
}

static void test_buffer_variant_matches_file_variant(void)
{
    /*
     * The buffer-based extractor is the WASM-build entry point (browser
     * has no FILE*). It must produce exactly the same HawkeyeTerrainParams
     * as the file-based version for the same .ulg.
     */
    FILE *fp = fopen(SIH_TERRAIN_LOG, "rb");
    assert(fp != NULL);
    fseek(fp, 0, SEEK_END);
    long sz = ftell(fp);
    assert(sz > 0);
    fseek(fp, 0, SEEK_SET);
    uint8_t *bytes = (uint8_t *)malloc((size_t)sz);
    assert(bytes != NULL);
    assert(fread(bytes, 1, (size_t)sz, fp) == (size_t)sz);
    fclose(fp);

    HawkeyeTerrainParams from_buf;
    bool ok = ulog_extract_terrain_params_from_buffer(bytes, (size_t)sz, &from_buf);
    free(bytes);
    assert(ok);
    assert(from_buf.mode == HAWKEYE_TERRAIN_MODE_TERRAIN);
    assert(approx_eq(from_buf.amp,     20.0f,  1e-3f));
    assert(approx_eq(from_buf.freq,    0.005f, 1e-6f));
    /* oct/hurst/erosion: hard-coded internals — not in the struct. */
    assert(from_buf.seed    == 3);
    printf("  PASS buffer_variant_matches_file_variant\n");
}

static void test_buffer_variant_rejects_bad_input(void)
{
    HawkeyeTerrainParams p;
    /* NULL buffer */
    assert(!ulog_extract_terrain_params_from_buffer(NULL, 0, &p));
    assert(p.mode == HAWKEYE_TERRAIN_MODE_OFF);
    /* Too short for header */
    uint8_t tiny[4] = { 'U', 'L', 'o', 'g' };
    assert(!ulog_extract_terrain_params_from_buffer(tiny, sizeof(tiny), &p));
    assert(p.mode == HAWKEYE_TERRAIN_MODE_OFF);
    /* Wrong magic */
    uint8_t bad_magic[16] = { 'B', 'A', 'D', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    assert(!ulog_extract_terrain_params_from_buffer(bad_magic, sizeof(bad_magic), &p));
    assert(p.mode == HAWKEYE_TERRAIN_MODE_OFF);
    printf("  PASS buffer_variant_rejects_bad_input\n");
}

static void test_apply_callback_fires(void)
{
    /*
     * Renderer fan-out: any apply call should invoke the registered
     * callback with the same pointer the underlying library got. This
     * is what scene/render-loop integration uses to keep the renderer's
     * local HawkeyeTerrainParams cache in sync with ingestion.
     */
    memset(&g_cb_capture, 0, sizeof(g_cb_capture));
    hawkeye_terrain_set_apply_callback(apply_callback);

    HawkeyeTerrainParams p = hawkeye_terrain_params_default();
    p.mode = HAWKEYE_TERRAIN_MODE_TERRAIN;
    p.amp  = 25.0f;
    p.freq = 0.005f;
    p.seed = 11;

    reset_stub();
    hawkeye_terrain_apply_params(&p);
    assert(g_cb_capture.calls == 1);
    assert(g_cb_capture.mode  == HAWKEYE_TERRAIN_MODE_TERRAIN);
    assert(g_cb_capture.amp   == 25.0f);
    assert(g_cb_capture.seed  == 11);

    /* NULL clears the registration — no further callbacks. */
    hawkeye_terrain_set_apply_callback(NULL);
    hawkeye_terrain_apply_params(&p);
    assert(g_cb_capture.calls == 1);
    printf("  PASS apply_callback_fires\n");
}

int main(void)
{
    printf("test_ulog_extract:\n");
    test_sih_terrain_on_log();
    test_apply_inverts_freq_to_wavelength();
    test_non_sih_log();
    test_missing_file();
    test_disabled_apply_forces_amp_zero();
    test_apply_null_uses_defaults();
    test_buffer_variant_matches_file_variant();
    test_buffer_variant_rejects_bad_input();
    test_apply_callback_fires();
    printf("ALL PASS\n");
    return 0;
}
