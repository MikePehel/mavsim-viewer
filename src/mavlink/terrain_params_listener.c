/**
 * @file terrain_params_listener.c
 *
 * Live-SIH PARAM_VALUE accumulator. See header for the contract.
 *
 * ## Value decoding
 *
 * PARAM_VALUE.param_value is always a float on the wire. For int-typed
 * parameters (SIH_TERR_EN, SIH_TERR_SEED) PX4's PARAM_SET
 * path encodes the integer set value as the IEEE-754 bit pattern of
 * the equivalent float, and the firmware mirrors that convention when
 * responding with PARAM_VALUE — so `param_value = 3.0f` means
 * "SIH_TERR_SEED = 3". Casting `(int)param_value` recovers the integer.
 *
 * This matches the data-section behaviour the ULog extractor handles
 * (see src/ulog/terrain_params_extract.c). The two ingestion paths
 * therefore share the same decoding rules.
 *
 * ## Concurrency
 *
 * Single-threaded — the MAVLink receive dispatch is on the main loop.
 * If a worker thread ever drives this directly, add a mutex around
 * the static state. The dirty flag is a single bool; reading and
 * clearing it under a lock would suffice.
 */

#include "terrain_params_listener.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <mavlink.h>   /* full definition of mavlink_param_value_t */

/* Wall-clock seconds via CLOCK_MONOTONIC. Used to stamp every accepted
 * PARAM_VALUE so the freshness-tracking layer above us can detect "this
 * value hasn't been refreshed in a while" without poking the network
 * code. Local to keep the listener self-contained. */
static double listener_now_seconds(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

/* -------------------------------------------------------------------------- */
/* Per-parameter book-keeping                                                  */
/* -------------------------------------------------------------------------- */

/* SIH_TERR_* set — AMP / FREQ restored on top of
 * wall lattice; both consume the same SEED. */
typedef enum {
    PARAM_EN = 0,
    PARAM_AMP,       /* SIH_TERR_AMP   [m]   fBm peak amplitude */
    PARAM_FREQ,      /* SIH_TERR_FREQ  [1/m] fBm base spatial frequency */
    PARAM_SEED,      /* SIH_TERR_SEED  [-]   integer hash seed */
    PARAM_PLANE,     /* SIH_TERR_PLANE [deg] planar slope (overrides fBm when nonzero) */
    PARAM_COUNT
} param_idx_t;

#define SEEN_ALL_MASK  ((1u << PARAM_COUNT) - 1u)   /* 0x1F */

/* Index -> upstream name. Used by the retry layer to issue
 * PARAM_REQUEST_READ for the still-missing entries; must stay in lock-
 * step with match_sih_terr_id() below. */
static const char *const k_param_names[PARAM_COUNT] = {
    "SIH_TERR_EN",
    "SIH_TERR_AMP",
    "SIH_TERR_FREQ",
    "SIH_TERR_SEED",
    "SIH_TERR_PLANE",
};

/* -------------------------------------------------------------------------- */
/* Module state                                                                */
/* -------------------------------------------------------------------------- */

static HawkeyeTerrainParams g_state;            /* accumulator */
static uint32_t              g_seen_mask;        /* bit per PARAM_* */
static bool                  g_initial_done;     /* set after first apply */
static bool                  g_dirty;            /* consumed by integration */
static bool                  g_state_initialised;
/* CLOCK_MONOTONIC seconds at which each PARAM_* was last seen on the
 * wire. 0 means "never". Drives the receiver's stale-detection log so
 * a silently-failing refresh loop is visible instead of invisible. */
static double                g_last_received[PARAM_COUNT];

static void ensure_initialised(void)
{
    if (!g_state_initialised) {
        g_state = hawkeye_terrain_params_default();
        g_seen_mask = 0;
        g_initial_done = false;
        g_dirty = false;
        for (int i = 0; i < (int)PARAM_COUNT; i++) {
            g_last_received[i] = 0.0;
        }
        g_state_initialised = true;
    }
}

/* -------------------------------------------------------------------------- */
/* Public helpers                                                              */
/* -------------------------------------------------------------------------- */

void terrain_params_listener_reset(void)
{
    g_state = hawkeye_terrain_params_default();
    g_seen_mask = 0;
    g_initial_done = false;
    g_dirty = false;
    for (int i = 0; i < (int)PARAM_COUNT; i++) {
        g_last_received[i] = 0.0;
    }
    g_state_initialised = true;
}

int terrain_params_listener_param_count(void)
{
    return (int)PARAM_COUNT;
}

unsigned int terrain_params_listener_missing_mask(void)
{
    ensure_initialised();
    return (~g_seen_mask) & SEEN_ALL_MASK;
}

const char *terrain_params_listener_name_for_index(int idx)
{
    if (idx < 0 || idx >= (int)PARAM_COUNT) {
        return NULL;
    }
    return k_param_names[idx];
}

double terrain_params_listener_seconds_since_received(int idx)
{
    if (idx < 0 || idx >= (int)PARAM_COUNT) return -1.0;
    ensure_initialised();
    if (g_last_received[idx] == 0.0) return -1.0;   /* never received */
    return listener_now_seconds() - g_last_received[idx];
}

const HawkeyeTerrainParams *terrain_params_listener_current(void)
{
    ensure_initialised();
    return &g_state;
}

bool terrain_params_listener_consume_dirty(void)
{
    if (!g_dirty) return false;
    g_dirty = false;
    return true;
}

/* -------------------------------------------------------------------------- */
/* PARAM_VALUE handling                                                        */
/* -------------------------------------------------------------------------- */

/*
 * Match `id` (NUL-terminated) against the five SIH_TERR_*
 * names. Returns the matching param_idx_t, or PARAM_COUNT if no match.
 */
static param_idx_t match_sih_terr_id(const char *id)
{
    if (strcmp(id, "SIH_TERR_EN")    == 0) return PARAM_EN;
    if (strcmp(id, "SIH_TERR_AMP")   == 0) return PARAM_AMP;
    if (strcmp(id, "SIH_TERR_FREQ")  == 0) return PARAM_FREQ;
    if (strcmp(id, "SIH_TERR_SEED")  == 0) return PARAM_SEED;
    if (strcmp(id, "SIH_TERR_PLANE") == 0) return PARAM_PLANE;
    return PARAM_COUNT;
}

/*
 * PX4 wire-encodes int32 parameters in PARAM_VALUE as the IEEE-754
 * bit pattern of the equivalent float (so int 1 lands as 0x00000001
 * which reads back as ~1.4e-45, NOT as 1.0f). The correct decode is
 * to reinterpret the float's bytes as int32_t — `(int)fval` would
 * truncate ~1.4e-45 to 0, which is wrong.
 */
static int decode_int_bits(float fval)
{
    int32_t out;
    memcpy(&out, &fval, sizeof(out));
    return (int)out;
}

/*
 * Apply the decoded value into the accumulator. Returns true if the
 * stored value actually changed.
 */
static bool store_value(param_idx_t idx, float fval)
{
    int ival = decode_int_bits(fval);
    bool changed = false;

    switch (idx) {
        case PARAM_EN: {
            /* SIH_TERR_EN is int32 enum (tri-state hotfix).
             * Values: 0=off, 1=terrain, 2=walls. Other values clamp to
             * OFF as a safety so a corrupted param can never render
             * garbage. */
            int new_mode = ival;
            if (new_mode < HAWKEYE_TERRAIN_MODE_OFF ||
                new_mode > HAWKEYE_TERRAIN_MODE_WALLS) {
                new_mode = HAWKEYE_TERRAIN_MODE_OFF;
            }
            if (new_mode != g_state.mode) {
                g_state.mode = new_mode;
                changed = true;
            }
            break;
        }
        case PARAM_AMP:
            /* AMP is a float on the wire — no bit-decode. */
            if (fval != g_state.amp) { g_state.amp = fval; changed = true; }
            break;
        case PARAM_FREQ:
            /* FREQ is a float on the wire — no bit-decode. */
            if (fval != g_state.freq) { g_state.freq = fval; changed = true; }
            break;
        case PARAM_SEED:
            if (ival != g_state.seed) { g_state.seed = ival; changed = true; }
            break;
        case PARAM_PLANE:
            if (fval != g_state.plane_deg) { g_state.plane_deg = fval; changed = true; }
            break;
        default:
            break;
    }
    return changed;
}

void terrain_params_on_param_value(const mavlink_param_value_t *msg)
{
    if (!msg) return;
    ensure_initialised();

    /*
     * param_id is char[16] and is NOT guaranteed NUL-terminated when
     * the id is exactly 16 chars. Copy into a 17-byte buffer with a
     * trailing NUL so strcmp() is safe. (The SIH_TERR_* names are all
     * <= 15 chars, so this only matters for hostile input.)
     */
    char id[17];
    memcpy(id, msg->param_id, sizeof(msg->param_id));
    id[sizeof(msg->param_id)] = '\0';

    param_idx_t idx = match_sih_terr_id(id);
    if (idx == PARAM_COUNT) return;

    /* Stamp the freshness timestamp for THIS name even if the value
     * didn't change — receipt itself is the freshness signal. */
    g_last_received[idx] = listener_now_seconds();

    bool changed   = store_value(idx, msg->param_value);
    bool was_seen  = (g_seen_mask & (1u << idx)) != 0u;
    g_seen_mask   |= (1u << idx);

    bool should_apply = false;
    if (!g_initial_done) {
        /*
         * Initial snapshot: apply once all five are in. Hawkeye should
         * never render a half-formed param set against the underlying
         * library — `terrain()` would mix stale defaults with the new
         * seed/amp until the last value lands.
         */
        if (g_seen_mask == SEEN_ALL_MASK) {
            should_apply = true;
            g_initial_done = true;
        }
    } else if (changed) {
        /* Post-snapshot: any genuine change pushes through immediately. */
        should_apply = true;
    }

    (void)was_seen;  /* tracked for future telemetry; not used in policy */

    if (should_apply) {
        hawkeye_terrain_apply_params(&g_state);
        g_dirty = true;
        printf("[terrain] applied: mode=%d amp=%.3f freq=%.4f seed=%d plane=%.3f\n",
               g_state.mode, g_state.amp, g_state.freq,
               g_state.seed, g_state.plane_deg);
    }
}
