/**
 * @file test_mavlink_listener.c
 *
 * Unit tests for terrain_params_on_param_value — live-SIH
 * PARAM_VALUE accumulator. Tests cover:
 *
 *   - initial-snapshot trigger fires exactly once when all 7 SIH_TERR_*
 *     have been seen, regardless of arrival order
 *   - post-snapshot value changes trigger an apply per change
 *   - re-receiving an unchanged value (post-snapshot) does NOT trigger
 *   - non-SIH_TERR_* messages are silently ignored
 *   - int-typed params (EN, OCT, SEED) decode via float-bit cast,
 *     matching PX4's PARAM_SET / PARAM_VALUE wire convention
 *   - NULL message pointer is safe (no-op)
 *   - getter returns latest accumulator
 *   - dirty flag is one-shot
 *
 * Like test_ulog_extract, this provides its own stub for
 * terrain_set_params() so it runs without the vendored terrain
 * library at link time.
 */

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <mavlink.h>   /* full mavlink_param_value_t definition */

#include "terrain/terrain_params.h"
#include "mavlink/terrain_params_listener.h"

/* --- Stub for the vendored terrain library ------------------------------- */

static struct {
    int   calls;
    float amp, wavelength;
    int   seed;
    float home_n, home_e;
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

static void reset_all(void)
{
    memset(&g_set_params, 0, sizeof(g_set_params));
    terrain_params_listener_reset();
    (void)terrain_params_listener_consume_dirty();  /* drain */
}

/* --- Helpers ------------------------------------------------------------- */

static void send_param(const char *id, float value)
{
    mavlink_param_value_t pv;
    memset(&pv, 0, sizeof(pv));
    /*
     * param_id is char[16] WITHOUT a NUL terminator when the id is
     * exactly 16 chars. Copy at most 16 bytes; trailing zeros from the
     * memset above pad the rest for shorter ids.
     */
    size_t n = strlen(id);
    if (n > sizeof(pv.param_id)) n = sizeof(pv.param_id);
    memcpy(pv.param_id, id, n);
    pv.param_value = value;
    pv.param_count = 7;
    pv.param_index = 0;
    pv.param_type  = MAV_PARAM_TYPE_REAL32;
    terrain_params_on_param_value(&pv);
}

/*
 * PX4 wire-encodes int32 params (SIH_TERR_EN, SIH_TERR_SEED) by writing
 * the int's bit pattern into the param_value float field — so PARAM_VALUE
 * for "SEED=3" carries the float whose IEEE-754 bits are 0x00000003 (a
 * denormal ~4.2e-45). The listener's decode_int_bits memcpys those bits
 * back to int32 to recover the integer.
 *
 * Test stubs must mimic the same encoding or the listener's int decode
 * lands on garbage. Use this helper for int params; use send_param() for
 * floats (AMP, FREQ, PLANE).
 */
static void send_param_int(const char *id, int32_t value)
{
    float as_float;
    memcpy(&as_float, &value, sizeof(as_float));
    mavlink_param_value_t pv;
    memset(&pv, 0, sizeof(pv));
    size_t n = strlen(id);
    if (n > sizeof(pv.param_id)) n = sizeof(pv.param_id);
    memcpy(pv.param_id, id, n);
    pv.param_value = as_float;
    pv.param_count = 7;
    pv.param_index = 0;
    pv.param_type  = MAV_PARAM_TYPE_INT32;
    terrain_params_on_param_value(&pv);
}

static bool approx_eq(float a, float b, float tol)
{
    return fabsf(a - b) <= tol;
}

/* --- Tests --------------------------------------------------------------- */

static void test_no_apply_before_full_snapshot(void)
{
    reset_all();
    /* Feed 4 of the 5 ( EN/AMP/FREQ/SEED/PLANE) — should not
     * trigger apply yet. */
    send_param("SIH_TERR_AMP",     20.0f);
    send_param("SIH_TERR_FREQ",    0.005f);
    send_param_int("SIH_TERR_SEED", 3);
    send_param("SIH_TERR_PLANE",   0.0f);
    assert(g_set_params.calls == 0);
    assert(!terrain_params_listener_consume_dirty());
    /* The 5th completes the snapshot and triggers apply.
     * EN=1 ( TERRAIN mode) → apply pushes the AMP through. */
    send_param_int("SIH_TERR_EN", 1);
    assert(g_set_params.calls == 1);
    assert(approx_eq(g_set_params.amp,        20.0f, 1e-3f));
    assert(approx_eq(g_set_params.wavelength, 200.0f, 1e-3f));
    assert(g_set_params.seed == 3);
    /* Dirty flag fired exactly once for the apply. */
    assert(terrain_params_listener_consume_dirty() == true);
    assert(terrain_params_listener_consume_dirty() == false);
    printf("  PASS no_apply_before_full_snapshot\n");
}

static void test_initial_snapshot_in_any_order(void)
{
    reset_all();
    /* Same 5 in reverse order — still triggers exactly once on completion. */
    send_param_int("SIH_TERR_EN", 1);
    send_param("SIH_TERR_PLANE",   0.0f);
    send_param_int("SIH_TERR_SEED", 3);
    send_param("SIH_TERR_FREQ",    0.005f);
    assert(g_set_params.calls == 0);
    send_param("SIH_TERR_AMP",     20.0f);
    assert(g_set_params.calls == 1);
    printf("  PASS initial_snapshot_in_any_order\n");
}

static void test_change_after_snapshot_triggers(void)
{
    reset_all();
    /* Complete the snapshot first — set is EN/AMP/FREQ/SEED/PLANE.
     * EN and SEED are int-typed on the PX4 wire; the rest are float. */
    send_param_int("SIH_TERR_EN", 1);   /* TERRAIN mode */
    send_param    ("SIH_TERR_AMP",   20.0f);
    send_param    ("SIH_TERR_FREQ",  0.005f);
    send_param_int("SIH_TERR_SEED", 3);
    send_param    ("SIH_TERR_PLANE", 0.0f);
    assert(g_set_params.calls == 1);
    /* Drain the dirty flag set by the initial apply. */
    assert(terrain_params_listener_consume_dirty());

    /* Same value re-sent: no extra apply, dirty stays false. */
    send_param_int("SIH_TERR_SEED", 3);
    assert(g_set_params.calls == 1);
    assert(!terrain_params_listener_consume_dirty());

    /* Change the seed: one more apply, dirty re-arms. */
    send_param_int("SIH_TERR_SEED", 7);
    assert(g_set_params.calls == 2);
    assert(g_set_params.seed == 7);
    assert(terrain_params_listener_consume_dirty());

    /* Change mode to OFF: amp gets forced to 0 by the apply layer. */
    send_param_int("SIH_TERR_EN", 0);
    assert(g_set_params.calls == 3);
    assert(g_set_params.amp == 0.0f);
    /* Wavelength still reflects the configured freq, not zeroed. */
    assert(approx_eq(g_set_params.wavelength, 200.0f, 1e-3f));
    printf("  PASS change_after_snapshot_triggers\n");
}

static void test_non_sih_params_ignored(void)
{
    reset_all();
    send_param("MAV_SYS_ID",   1.0f);
    send_param("MPC_ALT_MODE", 1.0f);
    send_param("SIH_LOC_LAT0", 47.0f);
    assert(g_set_params.calls == 0);
    assert(!terrain_params_listener_consume_dirty());
    printf("  PASS non_sih_params_ignored\n");
}

static void test_null_message_is_safe(void)
{
    reset_all();
    terrain_params_on_param_value(NULL);
    assert(g_set_params.calls == 0);
    printf("  PASS null_message_is_safe\n");
}

static void test_current_getter_reflects_latest(void)
{
    reset_all();
    /* Complete snapshot — set is EN/AMP/FREQ/SEED/PLANE. */
    send_param_int("SIH_TERR_EN", 1);   /* TERRAIN mode */
    send_param    ("SIH_TERR_AMP",   20.0f);
    send_param    ("SIH_TERR_FREQ",  0.005f);
    send_param_int("SIH_TERR_SEED", 3);
    send_param    ("SIH_TERR_PLANE", 0.0f);

    const HawkeyeTerrainParams *cur = terrain_params_listener_current();
    assert(cur != NULL);
    assert(cur->mode == HAWKEYE_TERRAIN_MODE_TERRAIN);  /* EN=1 → TERRAIN */
    assert(cur->seed == 3);
    assert(approx_eq(cur->amp, 20.0f, 1e-3f));

    /* After a change, getter shows the new value. */
    send_param_int("SIH_TERR_SEED", 42);
    assert(cur->seed == 42);  /* same pointer, fresh data */
    printf("  PASS current_getter_reflects_latest\n");
}

static void test_int_param_value_via_float_bits(void)
{
    /*
     * PARAM_VALUE always carries a float. For int params, the float
     * IS the integer value (e.g. PARAM_VALUE.param_value = 3.0f
     * means SIH_TERR_SEED = 3). The accumulator must round-trip this
     * correctly.
     */
    reset_all();
    send_param_int("SIH_TERR_EN",      1);   /* TERRAIN mode, int-typed wire */
    send_param    ("SIH_TERR_AMP",     20.0f);
    send_param    ("SIH_TERR_FREQ",    0.005f);
    send_param_int("SIH_TERR_SEED",    42);  /* int-typed */
    send_param    ("SIH_TERR_PLANE",   0.0f);
    assert(g_set_params.calls == 1);
    assert(g_set_params.seed == 42);
    printf("  PASS int_param_value_via_float_bits\n");
}

int main(void)
{
    printf("test_mavlink_listener:\n");
    test_no_apply_before_full_snapshot();
    test_initial_snapshot_in_any_order();
    test_change_after_snapshot_triggers();
    test_non_sih_params_ignored();
    test_null_message_is_safe();
    test_current_getter_reflects_latest();
    test_int_param_value_via_float_bits();
    printf("ALL PASS\n");
    return 0;
}
